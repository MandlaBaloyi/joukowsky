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
//!     the head difference.

use crate::HydraulicsError;
use crate::boundary::check_valve::CheckValveState;
use crate::boundary::junction::PipeEnd;
use crate::boundary::pump::PumpCurve;
use crate::moc::network::initial::SteadyState;
use crate::model::{Network, Node, NodeId, Topology};
use crate::pipe::friction::quasi_steady;
use crate::steady::{SteadyOptions, SteadySolution};
use jk_kernel::linalg::cholesky_solve;
use jk_kernel::{Flow, Head};

/// Largest number of free-head nodes the dense solver accepts (cost grows
/// as n^3; 2000 nodes is about a second per round on a laptop).
pub(crate) const MAX_UNKNOWN_HEADS: usize = 2000;
/// Flow below which a pipe's linearisation uses this flow instead [m^3/s].
const Q_FLOOR: f64 = 1e-5;
/// Multiple of machine epsilon allowed for rounding in a link's flow.
const ROUNDING_FACTOR: f64 = 64.0;
/// Smallest linearised resistance [s/m^2]. The answer does not depend on it
/// (only the path to it does), but conductance is 1/d and flow rounding
/// grows with it: a frictionless pipe with a tiny floor turns head rounding
/// into flow errors that a transient run would release as a fake wave.
const D_FLOOR: f64 = 1e-2;
/// Most passes allowed to settle the check valves' states.
const MAX_CHECK_VALVE_PASSES: usize = 20;

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
}

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
}

impl Graph {
    fn build(network: &Network, states: &[CheckValveState]) -> Result<Self, HydraulicsError> {
        let mut graph = Self {
            fixed_head: Vec::new(),
            demand: Vec::new(),
            origin: Vec::new(),
            links: Vec::new(),
            up_side: Vec::new(),
            down_side: Vec::new(),
            valve_link: vec![None; network.nodes().len()],
            pump_link: vec![None; network.nodes().len()],
        };
        let add = |g: &mut Self, fixed: Option<f64>, demand: f64, origin: usize| {
            g.fixed_head.push(fixed);
            g.demand.push(demand);
            g.origin.push(origin);
            g.fixed_head.len() - 1
        };
        for (n, node) in network.nodes().iter().enumerate() {
            let (fixed, demand) = match node {
                Node::Reservoir { head } => (Some(head.value()), 0.0),
                Node::Junction { demand } => (None, demand.value()),
                Node::Valve { .. } | Node::CheckValve | Node::Pump { .. } => (None, 0.0),
            };
            let up = add(&mut graph, fixed, demand, n);
            let split = matches!(node, Node::Pump { .. })
                || (matches!(node, Node::CheckValve) && states[n] == CheckValveState::Closed);
            let down = if split {
                add(&mut graph, None, 0.0, n)
            } else {
                up
            };
            graph.up_side.push(up);
            graph.down_side.push(down);
        }
        let g = network.gravity();
        for p in network.pipes() {
            let area = p.pipe.area().value();
            graph.links.push(Link {
                from: graph.down_side[p.from.0],
                to: graph.up_side[p.to.0],
                law: Law::Friction(quasi_steady::coefficient(
                    p.pipe.friction_factor(),
                    p.pipe.length(),
                    p.pipe.diameter(),
                    g,
                )),
                // EPANET's starting guess: about 1 ft/s in every pipe.
                initial_flow: 0.3 * area,
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
        }
        Ok(graph)
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
    /// shut-off head, or flow driven backwards); positive = guilty. When no
    /// solution exists Newton oscillates, so the last round alone is not a
    /// reliable witness.
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
    if m > MAX_UNKNOWN_HEADS {
        return Err(HydraulicsError::InvalidNetwork(format!(
            "{m} free-head nodes; the dense steady solver handles up to \
             {MAX_UNKNOWN_HEADS} (a sparse solver comes with municipal-scale networks)"
        ))
        .into());
    }

    let mut heads: Vec<f64> = graph.fixed_head.iter().map(|h| h.unwrap_or(0.0)).collect();
    let mut flows: Vec<f64> = graph.links.iter().map(|l| l.initial_flow).collect();
    let mut g = vec![0.0; graph.links.len()];
    let mut y = vec![0.0; graph.links.len()];
    let mut matrix = vec![0.0; m * m];
    let mut rhs = vec![0.0; m];
    let mut pump_badness = vec![f64::NEG_INFINITY; graph.links.len()];

    for iteration in 1..=options.max_iterations {
        matrix.fill(0.0);
        rhs.fill(0.0);
        for (j, link) in graph.links.iter().enumerate() {
            let q = flows[j];
            let (loss, d) = link.law.loss_and_slope(q);
            g[j] = 1.0 / d;
            y[j] = q - loss * g[j];
            let (a, b) = (link.from, link.to);
            if let Some(ia) = unknown[a] {
                matrix[ia * m + ia] += g[j];
                rhs[ia] -= y[j];
                match unknown[b] {
                    Some(ib) => matrix[ia * m + ib] -= g[j],
                    None => rhs[ia] += g[j] * heads[b],
                }
            }
            if let Some(ib) = unknown[b] {
                matrix[ib * m + ib] += g[j];
                rhs[ib] += y[j];
                match unknown[a] {
                    Some(ia) => matrix[ib * m + ia] -= g[j],
                    None => rhs[ib] += g[j] * heads[a],
                }
            }
        }
        for (i, &u) in unknown.iter().enumerate() {
            if let Some(k) = u {
                rhs[k] -= graph.demand[i];
            }
        }
        if m > 0 {
            cholesky_solve(&mut matrix, &mut rhs, m).map_err(|row| {
                let node = unknown
                    .iter()
                    .position(|&u| u == Some(row))
                    .map_or(0, |i| graph.origin[i]);
                HydraulicsError::InvalidNetwork(format!(
                    "heads cannot be determined near node {node}: part of the \
                     network does not reach any reservoir"
                ))
            })?;
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
    let nodes = network.nodes();
    let mut states: Vec<CheckValveState> = nodes
        .iter()
        .map(|n| match n {
            Node::CheckValve | Node::Pump { .. } => CheckValveState::Open,
            _ => CheckValveState::Closed,
        })
        .collect();

    for _ in 0..MAX_CHECK_VALVE_PASSES {
        let graph = Graph::build(network, &states)?;
        let solved = match newton(&graph, options) {
            Ok(solved) => solved,
            Err(Failure::Error(e)) => return Err(e),
            Err(Failure::NoConvergence { pump_badness }) => {
                // A running pump facing more head than its shut-off head, or
                // being driven backwards, has no forward operating point:
                // its non-return valve shuts. Shut the worst one and retry.
                let culprit = graph
                    .pump_link
                    .iter()
                    .enumerate()
                    .filter_map(|(n, link)| link.map(|j| (n, pump_badness[j])))
                    .filter(|&(_, badness)| badness > 0.0)
                    .max_by(|a, b| a.1.total_cmp(&b.1));
                if let Some((n, _)) = culprit {
                    states[n] = CheckValveState::Closed;
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
        if !changed {
            return assemble(network, topo, &graph, &solved, states);
        }
    }
    Err(HydraulicsError::InvalidNetwork(
        "check valves or pumps keep switching: no consistent steady state".into(),
    ))
}

#[allow(clippy::too_many_lines)] // one assembly, kept together for clarity
fn assemble(
    network: &Network,
    topo: &Topology,
    graph: &Graph,
    solved: &Solved,
    states: Vec<CheckValveState>,
) -> Result<(SteadySolution, SteadyState), HydraulicsError> {
    let nodes = network.nodes();
    let pipes = network.pipes();
    let heads = &solved.heads;
    let node_heads: Vec<f64> = graph.up_side.iter().map(|&s| heads[s]).collect();
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
    let pipe_end_heads: Vec<(f64, f64)> = pipes
        .iter()
        .map(|p| {
            (
                heads[graph.down_side[p.from.0]],
                heads[graph.up_side[p.to.0]],
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

    // How well the answer satisfies the equations it solved.
    let g = network.gravity();
    let mut energy = 0.0_f64;
    for (p, (q, (h_from, h_to))) in pipes.iter().zip(pipe_flows.iter().zip(&pipe_end_heads)) {
        let loss = p.pipe.friction_head_loss(Flow::new(*q), g).value();
        energy = energy.max((loss - (h_from - h_to)).abs());
    }
    // Continuity at every node that neither stores nor releases water:
    // junctions (net inflow = demand), pumps and check valves (in = out).
    let mut continuity = 0.0_f64;
    for (n, node) in nodes.iter().enumerate() {
        let demand = match node {
            Node::Junction { demand } => demand.value(),
            Node::Pump { .. } | Node::CheckValve => 0.0,
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
                Head::new(heads[graph.down_side[n]] - heads[graph.up_side[n]]),
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
    };
    let state = SteadyState {
        node_heads,
        pipe_flows,
        pipe_end_heads,
        valve_steady,
        check_valves: states,
    };
    Ok((solution, state))
}
