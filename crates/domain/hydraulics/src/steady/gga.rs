//! The Global Gradient Algorithm (Todini & Pilati 1988), as in EPANET.
//!
//! Unknowns: the flow Q in every link and the head H at every node whose
//! head is not fixed. Each link obeys the energy equation
//!
//! ```text
//!   H_from - H_to = R Q|Q|          (Darcy friction; R per pipe)
//! ```
//!
//! and each free node obeys continuity (inflow - outflow = demand).
//! Newton's method on both at once, with the energy equation linearised
//! around the current flow (d = 2 R |Q|, g = 1/d):
//!
//! ```text
//!   Q_new = y + g (H_from - H_to),   y = Q - R Q|Q| g
//! ```
//!
//! Substituting into continuity leaves one symmetric positive-definite
//! system for the heads (the "nodal matrix"); solve it, update every flow,
//! repeat. Convergence is quadratic near the solution: typically 5-10
//! rounds. The fixed point satisfies R Q|Q| = dH exactly whatever d is, so
//! the floor on d (for pipes with no flow or no friction) only affects the
//! path, never the answer.
//!
//! Devices:
//!   - reservoir: fixed-head node;
//!   - valve with flow coefficient Kv: a pseudo-link from the valve node to
//!     a fixed-head node at `H_down`, with R = 1 / Kv^2 (since
//!     `H - H_down` = Q^2 / Kv^2); Kv = 0 means shut (no link);
//!   - check valve: one node when open; split into two nodes (upstream and
//!     downstream side) when shut. The states are found by solving, shutting
//!     any open valve whose flow comes out negative, opening any shut valve
//!     whose upstream head exceeds its downstream head, and solving again;
//!   - pump (at rated speed): always two nodes (suction, discharge) joined by
//!     a link whose "loss" is the NEGATIVE of the pump curve,
//!     h(Q) = -(a0 + a1 Q + a2 Q^2), linearised with d = -dH/dQ. Its
//!     non-return valve is settled like a check valve: shut if the flow
//!     comes out negative, reopened if the shut-off head a0 could overcome
//!     the head difference;
//!   - control valve: two nodes (`boundary/control_valve.rs`). ACTIVE PRV:
//!     the downstream side is a fixed head at the setting, and a
//!     zero-conductance link carries the flow the downstream side sends on
//!     (updated every Newton round); ACTIVE PSV: the same with the UPSTREAM
//!     side fixed, the link carrying what arrives there; ACTIVE FCV: a link
//!     of fixed flow; OPEN (and a TCV, always): a friction link; CLOSED: no
//!     link. States settled with the check valves (EPANET's rules).
//!
//! Pressure-driven demand ([`PressureDemand`]): each junction with demand
//! is FULL (it draws its demand), CUT (it draws nothing) or PARTIAL - then
//! its demand is a link to a fixed head at ground + `p_min` whose loss is
//! the inverse of EPANET 2.2's law, (`p_req` - `p_min`) (Q / D)^(1/exponent).
//! Settled like the check valves: FULL goes PARTIAL below `p_req`, CUT goes
//! PARTIAL above `p_min`, PARTIAL goes FULL at its full demand and CUT at no
//! flow - so the answer is exact on both sides of the curve.
//!
//! Links out of service ([`Shut`]): a closed pipe stays in the graph with
//! no conductance (it carries nothing whatever the heads); a stopped pump
//! has no link, like a pump whose non-return valve is shut, and stays that
//! way. Closing links can cut part of the network off from every source:
//! if anyone there draws water, supply is lost there (an error, as for any
//! part that reaches no reservoir); if not, its heads are unknown - the
//! trapped water keeps whatever pressure it had - and are reported as NaN.

use crate::HydraulicsError;
use crate::boundary::check_valve::CheckValveState;
use crate::boundary::control_valve::{ValveMode, ValveState};
use crate::boundary::junction::PipeEnd;
use crate::boundary::pump::PumpCurve;
use crate::moc::network::initial::SteadyState;
use crate::model::{Network, Node, NodeId, Topology};
use crate::pipe::friction::quasi_steady;
use crate::steady::{PressureDemand, Shut, SteadyOptions, SteadySolution};
use jk_kernel::sparse::Envelope;
use jk_kernel::{Flow, Head, Time};

/// Largest nodal-matrix envelope accepted (entries stored): a guard that
/// turns a mistaken setup into an error instead of running out of memory
/// (2e8 entries = 1.6 GB).
const MAX_ENVELOPE: usize = 200_000_000;
/// Flow below which a pipe's linearisation uses this flow instead [m^3/s].
const Q_FLOOR: f64 = 1e-5;
/// Multiple of machine epsilon allowed for rounding in a link's flow.
const ROUNDING_FACTOR: f64 = 64.0;
/// Smallest linearised resistance [s/m^2]. The answer does not depend on it
/// (only the path to it does), but conductance is 1/d and flow rounding
/// grows with it: a frictionless pipe with a tiny floor turns head rounding
/// into flow errors that a transient run would release as a fake wave.
const D_FLOOR: f64 = 1e-2;
/// Most passes allowed to settle the check valves' and PRVs' states.
const MAX_CHECK_VALVE_PASSES: usize = 40;
/// Head tolerance for switching a control valve's state [m] (EPANET:
/// 0.0005 ft).
const VALVE_HEAD_TOL: f64 = 1e-4;
/// Reverse flow that shuts a one-way valve, and the margin by which an
/// open FCV must exceed its setting to become active [m^3/s].
const VALVE_FLOW_TOL: f64 = 1e-9;

/// How a link's head change depends on its flow.
#[derive(Clone, Copy)]
enum Law {
    /// Loss R Q|Q| (pipes; valves with R = 1/Kv^2).
    Friction(f64),
    /// Gain from a pump at rated speed: loss = -H(Q). The second value is
    /// the smallest slope used in the linearisation: the curve's slope at
    /// the rated flow. Near shut-off many curves rise slightly (a "hump"),
    /// where the true slope has the wrong sign; since the solution does not
    /// depend on the slope used, a safe positive one is taken instead.
    Pump(PumpCurve, f64),
    /// Out of service: no flow whatever the heads (conductance 0).
    Closed,
    /// An active PRV or PSV: no conductance; its flow is set each round
    /// from the other links at its fixed-head side `at`: `sign` times what
    /// they carry away from it (PRV +1: what the downstream side sends on;
    /// PSV -1: what arrives at the upstream side).
    Regulated { at: usize, sign: f64 },
    /// An active FCV: its flow is its setting (the link's initial flow).
    FixedFlow,
    /// A partially supplied junction's demand: loss span (Q / full)^(1 /
    /// exponent) to its outlet at ground + `p_min`; continued straight beyond
    /// 0 and full (the settling moves the junction off those).
    Pda { full: f64, span: f64, exponent: f64 },
}

/// How much of its demand a junction gets (pressure-driven demand).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Supply {
    /// All of it.
    Full,
    /// Some, by the pressure law.
    Partial,
    /// None.
    Cut,
}

/// Pressure tolerance for moving a junction between FULL, PARTIAL and CUT
/// [m].
const SUPPLY_TOL: f64 = 1e-6;

impl Law {
    /// Head LOST from `from` to `to` at flow q, and its derivative (floored).
    #[inline]
    fn loss_and_slope(&self, q: f64) -> (f64, f64) {
        match *self {
            Self::Friction(r) => (
                r * q * q.abs(),
                (2.0 * r * q.abs()).max(2.0 * r * Q_FLOOR).max(D_FLOOR),
            ),
            Self::Pump(curve, min_slope) => {
                (-curve.head(q, 1.0), (-curve.slope(q, 1.0)).max(min_slope))
            }
            Self::Closed | Self::Regulated { .. } | Self::FixedFlow => (0.0, f64::INFINITY),
            Self::Pda {
                full,
                span,
                exponent,
            } => {
                let (x, p) = (q / full, 1.0 / exponent);
                if x <= 0.0 {
                    (D_FLOOR * q, D_FLOOR)
                } else if x >= 1.0 {
                    let slope = span * p / full;
                    (span + slope * (q - full), slope)
                } else {
                    (
                        span * x.powf(p),
                        (span * p * x.powf(p - 1.0) / full).max(D_FLOOR),
                    )
                }
            }
        }
    }
}

/// A link of the solver graph: a pipe, a valve's pseudo-link, or a pump.
struct Link {
    from: usize,
    to: usize,
    law: Law,
    initial_flow: f64,
}

/// The solver graph for one set of check-valve states.
struct Graph {
    /// Known head of each solver node (reservoirs, valve outlets).
    fixed_head: Vec<Option<f64>>,
    /// Demand at each solver node.
    demand: Vec<f64>,
    /// Network node each solver node belongs to (for error messages).
    origin: Vec<usize>,
    /// Pipes first (link index = pipe index), then valve pseudo-links.
    links: Vec<Link>,
    /// Solver node receiving the pipes that END at each network node.
    up_side: Vec<usize>,
    /// Solver node for the pipes that START at each network node.
    down_side: Vec<usize>,
    /// Pseudo-link and outlet head of each valve node with Kv > 0.
    valve_link: Vec<Option<(usize, f64)>>,
    /// Link of each running (delivering) pump node.
    pump_link: Vec<Option<usize>>,
    /// Link of each PRV node that is ACTIVE or OPEN.
    prv_link: Vec<Option<usize>>,
    /// Demand link of each partially supplied junction.
    pda_link: Vec<Option<usize>>,
    /// Solver nodes cut off from every source, with no demand: their heads
    /// are unknown (their fixed head of 0 is a placeholder).
    isolated: Vec<bool>,
}

impl Graph {
    #[allow(clippy::too_many_lines)] // one graph, device by device
    fn build(
        network: &Network,
        states: &[CheckValveState],
        valves: &[Option<ValveState>],
        supply: &[Option<Supply>],
        pressure: Option<PressureDemand>,
        shut: &Shut,
    ) -> Result<Self, HydraulicsError> {
        let active = |n: usize| valves[n] == Some(ValveState::Active);
        let mut graph = Self {
            fixed_head: Vec::new(),
            demand: Vec::new(),
            origin: Vec::new(),
            links: Vec::new(),
            up_side: Vec::new(),
            down_side: Vec::new(),
            valve_link: vec![None; network.nodes().len()],
            pump_link: vec![None; network.nodes().len()],
            prv_link: vec![None; network.nodes().len()],
            pda_link: vec![None; network.nodes().len()],
            isolated: Vec::new(),
        };
        let add = |g: &mut Self, fixed: Option<f64>, demand: f64, origin: usize| {
            g.fixed_head.push(fixed);
            g.demand.push(demand);
            g.origin.push(origin);
            g.fixed_head.len() - 1
        };
        for (n, node) in network.nodes().iter().enumerate() {
            let (fixed, demand) = match node {
                Node::Reservoir { head } => (Some(network.source_head(n, *head, Time::ZERO)), 0.0),
                Node::Junction { demand } => match supply[n] {
                    Some(Supply::Partial | Supply::Cut) => (None, 0.0),
                    _ => (None, demand.value()),
                },
                Node::ControlValve {
                    mode: ValveMode::Psv { setting },
                    ..
                } if active(n) => (Some(setting.value()), 0.0),
                Node::Valve { .. }
                | Node::CheckValve
                | Node::Pump { .. }
                | Node::ControlValve { .. } => (None, 0.0),
            };
            let up = add(&mut graph, fixed, demand, n);
            let split = matches!(node, Node::Pump { .. } | Node::ControlValve { .. })
                || (matches!(node, Node::CheckValve) && states[n] == CheckValveState::Closed);
            let down = match node {
                Node::ControlValve {
                    mode: ValveMode::Prv { setting },
                    ..
                } if active(n) => add(&mut graph, Some(setting.value()), 0.0, n),
                _ if split => add(&mut graph, None, 0.0, n),
                _ => up,
            };
            graph.up_side.push(up);
            graph.down_side.push(down);
        }
        let g = network.gravity();
        for (k, p) in network.pipes().iter().enumerate() {
            let area = p.pipe.area().value();
            let closed = shut.pipes.iter().any(|id| id.0 == k);
            graph.links.push(Link {
                from: graph.down_side[p.from.0],
                to: graph.up_side[p.to.0],
                law: if closed {
                    Law::Closed
                } else {
                    Law::Friction(quasi_steady::coefficient(
                        p.pipe.friction_factor(),
                        p.pipe.length(),
                        p.pipe.diameter(),
                        g,
                    ))
                },
                // EPANET's starting guess: about 1 ft/s in every pipe.
                initial_flow: if closed { 0.0 } else { 0.3 * area },
            });
        }
        for (n, node) in network.nodes().iter().enumerate() {
            if let Node::Valve {
                downstream_head,
                flow_coefficient,
                ..
            } = node
            {
                let Some(kv) = *flow_coefficient else {
                    return Err(HydraulicsError::InvalidNetwork(format!(
                        "node {n}: a valve needs a flow_coefficient for a steady solve"
                    )));
                };
                if kv > 0.0 {
                    let outlet = add(&mut graph, Some(downstream_head.value()), 0.0, n);
                    graph.links.push(Link {
                        from: graph.up_side[n],
                        to: outlet,
                        law: Law::Friction(1.0 / (kv * kv)),
                        initial_flow: kv,
                    });
                    graph.valve_link[n] = Some((graph.links.len() - 1, downstream_head.value()));
                }
            }
            if let Node::Pump { pump, .. } = node
                && states[n] == CheckValveState::Open
            {
                graph.links.push(Link {
                    from: graph.up_side[n],
                    to: graph.down_side[n],
                    law: Law::Pump(
                        pump.curve,
                        (-pump.curve.slope(pump.rated_flow.value(), 1.0)).max(D_FLOOR),
                    ),
                    initial_flow: pump.rated_flow.value(),
                });
                graph.pump_link[n] = Some(graph.links.len() - 1);
            }
            if let Node::ControlValve {
                mode,
                open_resistance,
                ..
            } = node
            {
                let (up, down) = (graph.up_side[n], graph.down_side[n]);
                let law = match (valves[n], mode) {
                    (Some(ValveState::Active), ValveMode::Prv { .. }) => Some((
                        Law::Regulated {
                            at: down,
                            sign: 1.0,
                        },
                        0.0,
                    )),
                    (Some(ValveState::Active), ValveMode::Psv { .. }) => {
                        Some((Law::Regulated { at: up, sign: -1.0 }, 0.0))
                    }
                    (Some(ValveState::Active), ValveMode::Fcv { flow }) => {
                        Some((Law::FixedFlow, flow.value()))
                    }
                    (Some(ValveState::Open), _) | (_, ValveMode::Tcv) => {
                        Some((Law::Friction(*open_resistance), 0.0))
                    }
                    _ => None,
                };
                if let Some((law, initial_flow)) = law {
                    graph.links.push(Link {
                        from: up,
                        to: down,
                        law,
                        initial_flow,
                    });
                    graph.prv_link[n] = Some(graph.links.len() - 1);
                }
            }
        }
        if let Some(p) = pressure {
            for (n, node) in network.nodes().iter().enumerate() {
                if let (Node::Junction { demand }, Some(Supply::Partial)) = (node, supply[n]) {
                    let z = network.elevation(NodeId(n)).value();
                    let outlet = add(&mut graph, Some(z + p.minimum.value()), 0.0, n);
                    graph.links.push(Link {
                        from: graph.up_side[n],
                        to: outlet,
                        law: Law::Pda {
                            full: demand.value(),
                            span: p.required.value() - p.minimum.value(),
                            exponent: p.exponent,
                        },
                        initial_flow: 0.5 * demand.value(),
                    });
                    graph.pda_link[n] = Some(graph.links.len() - 1);
                }
            }
        }
        graph.isolated = vec![false; graph.fixed_head.len()];
        if !shut.is_empty()
            || valves.contains(&Some(ValveState::Closed))
            || supply.contains(&Some(Supply::Cut))
        {
            graph.isolate()?;
        }
        Ok(graph)
    }

    /// Finds the solver nodes that no source reaches through links in
    /// service. Any with demand: supply is lost there (an error). The rest
    /// get a placeholder fixed head and every link touching them is closed.
    fn isolate(&mut self) -> Result<(), HydraulicsError> {
        let n = self.fixed_head.len();
        let mut neighbours: Vec<Vec<usize>> = vec![Vec::new(); n];
        for link in &self.links {
            if !matches!(link.law, Law::Closed) {
                neighbours[link.from].push(link.to);
                neighbours[link.to].push(link.from);
            }
        }
        let mut reached: Vec<bool> = self.fixed_head.iter().map(Option::is_some).collect();
        let mut queue: Vec<usize> = (0..n).filter(|&i| reached[i]).collect();
        while let Some(i) = queue.pop() {
            for &j in &neighbours[i] {
                if !reached[j] {
                    reached[j] = true;
                    queue.push(j);
                }
            }
        }
        for i in (0..n).filter(|&i| !reached[i]) {
            if self.demand[i] > 0.0 {
                return Err(HydraulicsError::InvalidNetwork(format!(
                    "heads cannot be determined near node {}: closed pipes, stopped \
                     pumps or shut valves cut it off from every source, and it draws water",
                    self.origin[i]
                )));
            }
            self.isolated[i] = true;
            self.fixed_head[i] = Some(0.0);
        }
        for link in &mut self.links {
            if self.isolated[link.from] || self.isolated[link.to] {
                link.law = Law::Closed;
                link.initial_flow = 0.0;
            }
        }
        Ok(())
    }
}

/// Heads per solver node, flows per link, rounds used.
struct Solved {
    heads: Vec<f64>,
    flows: Vec<f64>,
    iterations: u32,
}

/// Why a Newton solve stopped without an answer.
enum Failure {
    /// A real error (bad network) - report it.
    Error(HydraulicsError),
    /// No convergence. For every link, the worst evidence seen over ALL
    /// rounds that it is a pump unable to deliver (head against it above its
    /// shut-off head, or flow driven backwards) or an active PRV driven
    /// backwards; positive = guilty. When no solution exists Newton
    /// oscillates, so the last round alone is not a reliable witness.
    NoConvergence { pump_badness: Vec<f64> },
}

impl From<HydraulicsError> for Failure {
    fn from(e: HydraulicsError) -> Self {
        Self::Error(e)
    }
}

/// Newton iteration on one graph.
#[allow(clippy::too_many_lines)] // one iteration, kept together for clarity
fn newton(graph: &Graph, options: &SteadyOptions) -> Result<Solved, Failure> {
    let n = graph.fixed_head.len();
    if graph.fixed_head.iter().all(Option::is_none) {
        return Err(HydraulicsError::InvalidNetwork(
            "a steady solve needs at least one reservoir".into(),
        )
        .into());
    }
    let mut unknown = vec![None; n];
    let mut m = 0;
    for (i, fixed) in graph.fixed_head.iter().enumerate() {
        if fixed.is_none() {
            unknown[i] = Some(m);
            m += 1;
        }
    }
    // The nodal matrix couples two unknown heads only through a link
    // between them: sparse. Its pattern is fixed, so it is set up once
    // (reverse Cuthill-McKee, envelope storage - jk_kernel::sparse) and
    // refilled every round.
    let edges: Vec<(usize, usize)> = graph
        .links
        .iter()
        .filter_map(|l| Some((unknown[l.from]?, unknown[l.to]?)))
        .collect();
    let mut matrix = Envelope::new(m, &edges);
    if matrix.stored() > MAX_ENVELOPE {
        return Err(HydraulicsError::InvalidNetwork(format!(
            "{m} free-head nodes need a nodal matrix of {} entries - more than the {MAX_ENVELOPE} \
             allowed",
            matrix.stored()
        ))
        .into());
    }

    let mut heads: Vec<f64> = graph.fixed_head.iter().map(|h| h.unwrap_or(0.0)).collect();
    let mut flows: Vec<f64> = graph.links.iter().map(|l| l.initial_flow).collect();
    let mut g = vec![0.0; graph.links.len()];
    let mut y = vec![0.0; graph.links.len()];
    let mut rhs = vec![0.0; m];
    let mut pump_badness = vec![f64::NEG_INFINITY; graph.links.len()];
    // Each active PRV's or PSV's flow follows from the other links at its
    // fixed-head side (+ leaving, - arriving), times its sign.
    let regulated: Vec<(usize, Vec<(usize, f64)>)> = graph
        .links
        .iter()
        .enumerate()
        .filter_map(|(j, l)| match l.law {
            Law::Regulated { at, sign } => Some((j, at, sign)),
            _ => None,
        })
        .map(|(j, d, sign)| {
            let others = graph
                .links
                .iter()
                .enumerate()
                .filter(|&(k, _)| k != j)
                .filter_map(|(k, o)| {
                    if o.from == d {
                        Some((k, sign))
                    } else if o.to == d {
                        Some((k, -sign))
                    } else {
                        None
                    }
                })
                .collect();
            (j, others)
        })
        .collect();

    for iteration in 1..=options.max_iterations {
        matrix.clear();
        rhs.fill(0.0);
        for (j, link) in graph.links.iter().enumerate() {
            let q = flows[j];
            let (loss, d) = link.law.loss_and_slope(q);
            g[j] = 1.0 / d;
            y[j] = q - loss * g[j];
            let (a, b) = (link.from, link.to);
            if let Some(ia) = unknown[a] {
                matrix.add(ia, ia, g[j]);
                rhs[ia] -= y[j];
                match unknown[b] {
                    Some(ib) => matrix.add(ia, ib, -g[j]),
                    None => rhs[ia] += g[j] * heads[b],
                }
            }
            if let Some(ib) = unknown[b] {
                matrix.add(ib, ib, g[j]);
                rhs[ib] += y[j];
                if unknown[a].is_none() {
                    rhs[ib] += g[j] * heads[a];
                }
            }
        }
        for (i, &u) in unknown.iter().enumerate() {
            if let Some(k) = u {
                rhs[k] -= graph.demand[i];
            }
        }
        if m > 0 {
            matrix.factor().map_err(|row| {
                let node = unknown
                    .iter()
                    .position(|&u| u == Some(row))
                    .map_or(0, |i| graph.origin[i]);
                HydraulicsError::InvalidNetwork(format!(
                    "heads cannot be determined near node {node}: part of the \
                     network does not reach any reservoir"
                ))
            })?;
            matrix.solve(&mut rhs);
            for (i, &u) in unknown.iter().enumerate() {
                if let Some(k) = u {
                    heads[i] = rhs[k];
                }
            }
        }
        // Rounding floor: a link's flow is g * (head difference), and heads
        // carry rounding of ~eps * |H|. For a short pipe carrying almost no
        // flow g is huge, so its flow cannot be known better than
        // g * eps * |H| - demanding more would never converge. Each link is
        // judged against its own floor; ordinary pipes keep the full
        // tolerance.
        let head_scale = heads.iter().fold(1.0_f64, |m, h| m.max(h.abs()));
        let mut new_flows = Vec::with_capacity(flows.len());
        let mut scale = 0.0_f64;
        for (j, link) in graph.links.iter().enumerate() {
            let q_new = y[j] + g[j] * (heads[link.from] - heads[link.to]);
            scale = scale.max(q_new.abs());
            new_flows.push(q_new);
            if let Law::Pump(curve, _) = link.law {
                let against = heads[link.to] - heads[link.from];
                let excess = against - curve.coefficients().0;
                pump_badness[j] = pump_badness[j].max(excess).max(-q_new * 1e6);
            }
        }
        for (j, others) in &regulated {
            new_flows[*j] = others.iter().map(|&(k, sign)| sign * new_flows[k]).sum();
            scale = scale.max(new_flows[*j].abs());
            pump_badness[*j] = pump_badness[*j].max(-new_flows[*j] * 1e6);
        }
        let tol = options.tolerance * scale.max(1e-3);
        let converged = new_flows
            .iter()
            .zip(&flows)
            .zip(&g)
            .all(|((new, old), gj)| {
                (new - old).abs() <= tol + ROUNDING_FACTOR * f64::EPSILON * head_scale * gj
            });
        flows = new_flows;
        if converged {
            return Ok(Solved {
                heads,
                flows,
                iterations: iteration,
            });
        }
    }
    Err(Failure::NoConvergence { pump_badness })
}

/// Solves the steady state, settling check valves; returns the public
/// solution and the state the transient solver starts from.
pub(crate) fn solve(
    network: &Network,
    topo: &Topology,
    options: &SteadyOptions,
) -> Result<(SteadySolution, SteadyState), HydraulicsError> {
    solve_shut(network, topo, options, &Shut::default())
}

/// [`solve`] with some links out of service.
#[allow(clippy::too_many_lines)] // one settling loop over every device kind
pub(crate) fn solve_shut(
    network: &Network,
    topo: &Topology,
    options: &SteadyOptions,
    shut: &Shut,
) -> Result<(SteadySolution, SteadyState), HydraulicsError> {
    let nodes = network.nodes();
    let stopped = |n: usize| shut.pumps.iter().any(|id| id.0 == n);
    let mut states: Vec<CheckValveState> = nodes
        .iter()
        .enumerate()
        .map(|(n, node)| match node {
            Node::Pump { .. } if stopped(n) => CheckValveState::Closed,
            Node::CheckValve | Node::Pump { .. } => CheckValveState::Open,
            _ => CheckValveState::Closed,
        })
        .collect();
    let mut valves: Vec<Option<ValveState>> = nodes
        .iter()
        .map(|node| match node {
            Node::ControlValve {
                mode: ValveMode::Tcv,
                ..
            } => Some(ValveState::Open),
            Node::ControlValve { .. } => Some(ValveState::Active),
            _ => None,
        })
        .collect();
    let pressure = options.pressure_demand;
    // Junctions start PARTIAL: their outlets then give the solve a fixed
    // head even where no reservoir does (a network fed only by an empty
    // tank's trickle); the settling moves them to FULL or CUT.
    let mut supply: Vec<Option<Supply>> = nodes
        .iter()
        .map(|node| match node {
            Node::Junction { demand } if pressure.is_some() && demand.value() > 0.0 => {
                Some(Supply::Partial)
            }
            _ => None,
        })
        .collect();

    for _ in 0..MAX_CHECK_VALVE_PASSES {
        let graph = Graph::build(network, &states, &valves, &supply, pressure, shut)?;
        let solved = match newton(&graph, options) {
            Ok(solved) => solved,
            Err(Failure::Error(e)) => return Err(e),
            Err(Failure::NoConvergence { pump_badness }) => {
                // A running pump facing more head than its shut-off head, or
                // being driven backwards, has no forward operating point:
                // its non-return valve shuts. So does an active PRV driven
                // backwards (e.g. a second source holding its zone above
                // the setting through pipes with no friction). Shut the
                // worst one and retry.
                let active = |n: usize| valves[n] == Some(ValveState::Active);
                let culprit = graph
                    .pump_link
                    .iter()
                    .chain(&graph.prv_link)
                    .enumerate()
                    .filter_map(|(k, link)| link.map(|j| (k, pump_badness[j])))
                    .filter(|&(k, _)| k < nodes.len() || active(k - nodes.len()))
                    .filter(|&(_, badness)| badness > 0.0)
                    .max_by(|a, b| a.1.total_cmp(&b.1));
                if let Some((k, _)) = culprit {
                    if k < nodes.len() {
                        states[k] = CheckValveState::Closed;
                    } else {
                        valves[k - nodes.len()] = Some(ValveState::Closed);
                    }
                    continue;
                }
                return Err(HydraulicsError::InvalidNetwork(format!(
                    "steady solve did not converge in {} iterations",
                    options.max_iterations
                )));
            }
        };
        let mut changed = false;
        for (n, node) in nodes.iter().enumerate() {
            // Head a pump could add at zero flow (0 for a check valve).
            let push = match node {
                Node::CheckValve => 0.0,
                Node::Pump { .. } if stopped(n) => continue,
                Node::Pump { pump, .. } => pump.curve.coefficients().0,
                _ => continue,
            };
            match states[n] {
                CheckValveState::Open => {
                    let through = if let Some(link) = graph.pump_link[n] {
                        solved.flows[link]
                    } else {
                        let incoming = topo[n]
                            .iter()
                            .find(|(_, e)| *e == PipeEnd::Downstream)
                            .map(|(p, _)| p.0)
                            .expect("topology checked: one incoming pipe");
                        solved.flows[incoming]
                    };
                    if through < -1e-12 {
                        states[n] = CheckValveState::Closed;
                        changed = true;
                    }
                }
                CheckValveState::Closed => {
                    let (up, down) = (graph.up_side[n], graph.down_side[n]);
                    if solved.heads[up] + push > solved.heads[down] + 1e-9 {
                        states[n] = CheckValveState::Open;
                        changed = true;
                    }
                }
            }
        }
        for (n, node) in nodes.iter().enumerate() {
            let (Node::ControlValve { mode, .. }, Some(state)) = (node, valves[n]) else {
                continue;
            };
            let (up, down) = (
                solved.heads[graph.up_side[n]],
                solved.heads[graph.down_side[n]],
            );
            let q = graph.prv_link[n].map_or(0.0, |j| solved.flows[j]);
            let next = match mode {
                ValveMode::Prv { setting } => prv_next(state, up, down, q, setting.value()),
                ValveMode::Psv { setting } => psv_next(state, up, down, q, setting.value()),
                ValveMode::Fcv { flow } => fcv_next(state, up, down, q, flow.value()),
                ValveMode::Tcv => state,
            };
            if next != state {
                valves[n] = Some(next);
                changed = true;
            }
        }
        if let Some(p) = pressure {
            for (n, node) in nodes.iter().enumerate() {
                let (Node::Junction { demand }, Some(s)) = (node, supply[n]) else {
                    continue;
                };
                let up = graph.up_side[n];
                let head = if graph.isolated[up] {
                    f64::NAN
                } else {
                    solved.heads[up]
                };
                let pressure_head = head - network.elevation(NodeId(n)).value();
                let next = match s {
                    Supply::Full if pressure_head < p.required.value() - SUPPLY_TOL => {
                        Supply::Partial
                    }
                    Supply::Cut if pressure_head > p.minimum.value() + SUPPLY_TOL => {
                        Supply::Partial
                    }
                    Supply::Partial => {
                        let q = graph.pda_link[n].map_or(0.0, |j| solved.flows[j]);
                        if q >= demand.value() {
                            Supply::Full
                        } else if q <= 0.0 {
                            Supply::Cut
                        } else {
                            Supply::Partial
                        }
                    }
                    s => s,
                };
                if next != s {
                    supply[n] = Some(next);
                    changed = true;
                }
            }
        }
        if !changed {
            return assemble(network, topo, &graph, &solved, states, &valves, &supply);
        }
    }
    Err(HydraulicsError::InvalidNetwork(
        "check valves, pumps or control valves keep switching: no consistent steady state".into(),
    ))
}

/// A PRV's next state from its heads (upstream, downstream), its flow and
/// its setting - EPANET 2.2's rules (prvstatus) [VERIFY against its
/// source].
fn prv_next(state: ValveState, up: f64, down: f64, q: f64, set: f64) -> ValveState {
    let tol = VALVE_HEAD_TOL;
    match state {
        ValveState::Active | ValveState::Open if q < -VALVE_FLOW_TOL => ValveState::Closed,
        ValveState::Active if up < set - tol => ValveState::Open,
        ValveState::Open if down > set + tol => ValveState::Active,
        ValveState::Closed if up > set + tol && down < set - tol => ValveState::Active,
        ValveState::Closed if up < set - tol && up > down + tol => ValveState::Open,
        s => s,
    }
}

/// A PSV's next state - the mirror of a PRV: it holds the head UPSTREAM
/// at `set`, and needs no throttling once the head downstream is above it
/// (EPANET 2.2's psvstatus) [VERIFY against its source].
fn psv_next(state: ValveState, up: f64, down: f64, q: f64, set: f64) -> ValveState {
    let tol = VALVE_HEAD_TOL;
    match state {
        ValveState::Active | ValveState::Open if q < -VALVE_FLOW_TOL => ValveState::Closed,
        ValveState::Active if down > set + tol => ValveState::Open,
        ValveState::Open if up < set - tol => ValveState::Active,
        ValveState::Closed if down > set + tol && up > down + tol => ValveState::Open,
        ValveState::Closed if up > set + tol && up > down + tol => ValveState::Active,
        s => s,
    }
}

/// An FCV's next state: active it passes exactly `flow`; if that would
/// need the head to rise across it, it cannot - fully open, it passes what
/// it can; open, it becomes active once it would pass more than `flow`;
/// it never passes flow backwards [VERIFY against EPANET's fcvstatus].
fn fcv_next(state: ValveState, up: f64, down: f64, q: f64, flow: f64) -> ValveState {
    let tol = VALVE_HEAD_TOL;
    match state {
        ValveState::Active if up < down - tol => ValveState::Open,
        ValveState::Open if q < -VALVE_FLOW_TOL => ValveState::Closed,
        ValveState::Open if q > flow + VALVE_FLOW_TOL => ValveState::Active,
        ValveState::Closed if up > down + tol => ValveState::Open,
        s => s,
    }
}

#[allow(clippy::too_many_lines)] // one assembly, kept together for clarity
fn assemble(
    network: &Network,
    topo: &Topology,
    graph: &Graph,
    solved: &Solved,
    states: Vec<CheckValveState>,
    valves: &[Option<ValveState>],
    supply: &[Option<Supply>],
) -> Result<(SteadySolution, SteadyState), HydraulicsError> {
    let nodes = network.nodes();
    let pipes = network.pipes();
    let heads = &solved.heads;
    let head_of = |s: usize| {
        if graph.isolated[s] {
            f64::NAN
        } else {
            heads[s]
        }
    };
    let node_heads: Vec<f64> = graph.up_side.iter().map(|&s| head_of(s)).collect();
    let mut pipe_flows = solved.flows[..pipes.len()].to_vec();
    // A shut check valve passes nothing: that is its boundary condition, so
    // the pipes on both sides carry exactly zero - not the ~1e-11 m^3/s of
    // rounding the Newton solve leaves at a dead end.
    for (n, _) in nodes.iter().enumerate().filter(|(n, node)| {
        matches!(node, Node::CheckValve | Node::Pump { .. })
            && states[*n] == CheckValveState::Closed
    }) {
        for &(p, _) in &topo[n] {
            pipe_flows[p.0] = 0.0;
        }
    }
    for n in (0..nodes.len()).filter(|&n| valves[n] == Some(ValveState::Closed)) {
        for &(p, _) in &topo[n] {
            pipe_flows[p.0] = 0.0;
        }
    }
    let pipe_end_heads: Vec<(f64, f64)> = pipes
        .iter()
        .map(|p| {
            (
                head_of(graph.down_side[p.from.0]),
                head_of(graph.up_side[p.to.0]),
            )
        })
        .collect();

    let mut valve_steady = vec![(0.0, 1.0); nodes.len()];
    for (n, link) in graph.valve_link.iter().enumerate() {
        if let Some((j, outlet_head)) = *link {
            let (q0, drop) = (solved.flows[j], node_heads[n] - outlet_head);
            if q0 < 0.0 || (q0 > 0.0 && drop <= 0.0) {
                return Err(HydraulicsError::NoHeadDropAtValve { head_drop: drop });
            }
            if q0 > 0.0 {
                valve_steady[n] = (q0, drop);
            }
        }
    }
    // A control valve passing flow: its flow and head drop, so a transient
    // can hold it at that opening (a drop a hair below 0 within the state
    // tolerance is no drop).
    for (n, link) in graph.prv_link.iter().enumerate() {
        if let Some(j) = *link {
            let q0 = solved.flows[j];
            if q0 > VALVE_FLOW_TOL {
                let drop = head_of(graph.up_side[n]) - head_of(graph.down_side[n]);
                valve_steady[n] = (q0, drop.max(0.0));
            }
        }
    }

    // How well the answer satisfies the equations it solved.
    let g = network.gravity();
    let mut energy = 0.0_f64;
    for (k, (p, (q, (h_from, h_to)))) in pipes
        .iter()
        .zip(pipe_flows.iter().zip(&pipe_end_heads))
        .enumerate()
    {
        if matches!(graph.links[k].law, Law::Closed) {
            continue; // no flow, whatever the heads either side
        }
        let loss = p.pipe.friction_head_loss(Flow::new(*q), g).value();
        energy = energy.max((loss - (h_from - h_to)).abs());
    }
    // Demand asked and delivered at each node.
    let requested: Vec<f64> = nodes
        .iter()
        .map(|node| match node {
            Node::Junction { demand } => demand.value(),
            _ => 0.0,
        })
        .collect();
    let delivered: Vec<f64> = (0..nodes.len())
        .map(|n| match supply[n] {
            None | Some(Supply::Full) => requested[n],
            Some(Supply::Cut) => 0.0,
            Some(Supply::Partial) => {
                graph.pda_link[n].map_or(0.0, |j| solved.flows[j].clamp(0.0, requested[n]))
            }
        })
        .collect();
    // Continuity at every node that neither stores nor releases water:
    // junctions (net inflow = delivered demand), pumps and check valves (in
    // = out).
    let mut continuity = 0.0_f64;
    for (n, node) in nodes.iter().enumerate() {
        let demand = match node {
            Node::Junction { .. } => delivered[n],
            Node::Pump { .. } | Node::CheckValve | Node::ControlValve { .. } => 0.0,
            Node::Reservoir { .. } | Node::Valve { .. } => continue,
        };
        let net: f64 = topo[n]
            .iter()
            .map(|&(p, end)| match end {
                PipeEnd::Downstream => pipe_flows[p.0],
                PipeEnd::Upstream => -pipe_flows[p.0],
            })
            .sum();
        continuity = continuity.max((net - demand).abs());
    }

    let closed_check_valves = nodes
        .iter()
        .enumerate()
        .filter(|(n, node)| {
            matches!(node, Node::CheckValve) && states[*n] == CheckValveState::Closed
        })
        .map(|(n, _)| NodeId(n))
        .collect();
    let idle_pumps = nodes
        .iter()
        .enumerate()
        .filter(|(n, node)| {
            matches!(node, Node::Pump { .. }) && states[*n] == CheckValveState::Closed
        })
        .map(|(n, _)| NodeId(n))
        .collect();
    let pump_lifts = nodes
        .iter()
        .enumerate()
        .filter(|(_, node)| matches!(node, Node::Pump { .. }))
        .map(|(n, _)| {
            (
                NodeId(n),
                Head::new(head_of(graph.down_side[n]) - head_of(graph.up_side[n])),
            )
        })
        .collect();
    let solution = SteadySolution {
        node_heads: node_heads.iter().map(|&h| Head::new(h)).collect(),
        pipe_flows: pipe_flows.iter().map(|&q| Flow::new(q)).collect(),
        iterations: solved.iterations,
        max_energy_residual: Head::new(energy),
        max_continuity_residual: Flow::new(continuity),
        closed_check_valves,
        idle_pumps,
        pump_lifts,
        valves: valves
            .iter()
            .enumerate()
            .filter_map(|(n, s)| s.map(|s| (NodeId(n), s)))
            .collect(),
        requested: requested.iter().map(|&q| Flow::new(q)).collect(),
        delivered: delivered.iter().map(|&q| Flow::new(q)).collect(),
    };
    let state = SteadyState {
        node_heads,
        pipe_flows,
        pipe_end_heads,
        valve_steady,
        check_valves: states,
        junction_outflow: delivered,
    };
    Ok((solution, state))
}
