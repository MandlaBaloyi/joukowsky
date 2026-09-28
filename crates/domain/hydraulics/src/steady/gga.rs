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
//!     whose upstream head exceeds its downstream head, and solving again.

use crate::HydraulicsError;
use crate::boundary::check_valve::CheckValveState;
use crate::boundary::junction::PipeEnd;
use crate::moc::network::initial::SteadyState;
use crate::model::{Network, Node, NodeId, Topology};
use crate::pipe::friction::quasi_steady;
use crate::steady::dense::cholesky_solve;
use crate::steady::{SteadyOptions, SteadySolution};
use jk_kernel::{Flow, Head};

/// Largest number of free-head nodes the dense solver accepts (cost grows
/// as n^3; 2000 nodes is about a second per round on a laptop).
pub(crate) const MAX_UNKNOWN_HEADS: usize = 2000;
/// Flow below which a pipe's linearisation uses this flow instead [m^3/s].
const Q_FLOOR: f64 = 1e-7;
/// Smallest linearised resistance, for frictionless links [s/m^2].
const D_FLOOR: f64 = 1e-7;
/// Most passes allowed to settle the check valves' states.
const MAX_CHECK_VALVE_PASSES: usize = 20;

/// A link of the solver graph: a pipe, or a valve's pseudo-link.
struct Link {
    from: usize,
    to: usize,
    resistance: f64,
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
                Node::Valve { .. } | Node::CheckValve => (None, 0.0),
            };
            let up = add(&mut graph, fixed, demand, n);
            let down = if matches!(node, Node::CheckValve) && states[n] == CheckValveState::Closed {
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
                resistance: quasi_steady::coefficient(
                    p.pipe.friction_factor(),
                    p.pipe.length(),
                    p.pipe.diameter(),
                    g,
                ),
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
                        resistance: 1.0 / (kv * kv),
                        initial_flow: kv,
                    });
                    graph.valve_link[n] = Some((graph.links.len() - 1, downstream_head.value()));
                }
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

/// Newton iteration on one graph.
fn newton(graph: &Graph, options: &SteadyOptions) -> Result<Solved, HydraulicsError> {
    let n = graph.fixed_head.len();
    if graph.fixed_head.iter().all(Option::is_none) {
        return Err(HydraulicsError::InvalidNetwork(
            "a steady solve needs at least one reservoir".into(),
        ));
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
        )));
    }

    let mut heads: Vec<f64> = graph.fixed_head.iter().map(|h| h.unwrap_or(0.0)).collect();
    let mut flows: Vec<f64> = graph.links.iter().map(|l| l.initial_flow).collect();
    let mut g = vec![0.0; graph.links.len()];
    let mut y = vec![0.0; graph.links.len()];
    let mut matrix = vec![0.0; m * m];
    let mut rhs = vec![0.0; m];

    for iteration in 1..=options.max_iterations {
        matrix.fill(0.0);
        rhs.fill(0.0);
        for (j, link) in graph.links.iter().enumerate() {
            let q = flows[j];
            let r = link.resistance;
            let d = (2.0 * r * q.abs()).max(2.0 * r * Q_FLOOR).max(D_FLOOR);
            g[j] = 1.0 / d;
            y[j] = q - r * q * q.abs() * g[j];
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
        let (mut change, mut scale) = (0.0_f64, 0.0_f64);
        for (j, link) in graph.links.iter().enumerate() {
            let q_new = y[j] + g[j] * (heads[link.from] - heads[link.to]);
            change = change.max((q_new - flows[j]).abs());
            scale = scale.max(q_new.abs());
            flows[j] = q_new;
        }
        if change <= options.tolerance * scale.max(1e-3) {
            return Ok(Solved {
                heads,
                flows,
                iterations: iteration,
            });
        }
    }
    Err(HydraulicsError::InvalidNetwork(format!(
        "steady solve did not converge in {} iterations",
        options.max_iterations
    )))
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
            Node::CheckValve => CheckValveState::Open,
            _ => CheckValveState::Closed,
        })
        .collect();

    for _ in 0..MAX_CHECK_VALVE_PASSES {
        let graph = Graph::build(network, &states)?;
        let solved = newton(&graph, options)?;
        let mut changed = false;
        for (n, node) in nodes.iter().enumerate() {
            if !matches!(node, Node::CheckValve) {
                continue;
            }
            match states[n] {
                CheckValveState::Open => {
                    let incoming = topo[n]
                        .iter()
                        .find(|(_, e)| *e == PipeEnd::Downstream)
                        .map(|(p, _)| p.0)
                        .expect("topology checked: a check valve has an incoming pipe");
                    if solved.flows[incoming] < -1e-12 {
                        states[n] = CheckValveState::Closed;
                        changed = true;
                    }
                }
                CheckValveState::Closed => {
                    let (up, down) = (graph.up_side[n], graph.down_side[n]);
                    if solved.heads[up] > solved.heads[down] + 1e-9 {
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
        "check valves keep switching: no consistent steady state".into(),
    ))
}

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
        matches!(node, Node::CheckValve) && states[*n] == CheckValveState::Closed
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
    let mut continuity = 0.0_f64;
    for (n, node) in nodes.iter().enumerate() {
        if let Node::Junction { demand } = node {
            let net: f64 = topo[n]
                .iter()
                .map(|&(p, end)| match end {
                    PipeEnd::Downstream => pipe_flows[p.0],
                    PipeEnd::Upstream => -pipe_flows[p.0],
                })
                .sum();
            continuity = continuity.max((net - demand.value()).abs());
        }
    }

    let closed_check_valves = nodes
        .iter()
        .enumerate()
        .filter(|(n, node)| {
            matches!(node, Node::CheckValve) && states[*n] == CheckValveState::Closed
        })
        .map(|(n, _)| NodeId(n))
        .collect();
    let solution = SteadySolution {
        node_heads: node_heads.iter().map(|&h| Head::new(h)).collect(),
        pipe_flows: pipe_flows.iter().map(|&q| Flow::new(q)).collect(),
        iterations: solved.iterations,
        max_energy_residual: Head::new(energy),
        max_continuity_residual: Flow::new(continuity),
        closed_check_valves,
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
