//! The steady state a transient starts from.
//!
//! A transient simulation is only meaningful if it starts from a true
//! steady state; otherwise the solver "releases" the error as a fake
//! transient on the first step. Until the steady-state solver (milestone
//! S4) exists, two ways are offered - and both are CHECKED, never trusted:
//!
//! - [`InitialCondition::AtRest`]: everything full, nothing flowing, one
//!   head everywhere. Valid only if every reservoir has that head and no
//!   junction draws a demand. This is the classic setup for a burst in a
//!   pressurised network.
//! - [`InitialCondition::PipeFlows`]: the caller gives each pipe's flow;
//!   heads are found by walking out from the reservoirs, subtracting
//!   friction losses. Rejected if heads around a loop do not close, flows
//!   do not balance at a node, or a node is not connected to a reservoir.

use std::collections::VecDeque;

use jk_kernel::{Flow, Head};

use crate::HydraulicsError;
use crate::boundary::check_valve::CheckValveState;
use crate::boundary::junction::PipeEnd;
use crate::moc::network::model::{Network, Node, Topology};

/// Largest head mismatch accepted when walking around a loop [m].
const HEAD_TOLERANCE: f64 = 1e-6;
/// Largest flow imbalance accepted at a node, relative to the flows there.
const FLOW_TOLERANCE: f64 = 1e-9;

/// How the network is set up at t = 0.
#[derive(Debug, Clone, PartialEq)]
pub enum InitialCondition {
    /// Full of water at `head` everywhere, nothing flowing.
    AtRest {
        /// The common head.
        head: Head,
    },
    /// The steady flow in each pipe (indexed like [`Network::pipes`]),
    /// positive from the pipe's `from` node to its `to` node.
    PipeFlows(Vec<Flow>),
}

/// The resolved steady state, in solver units.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct SteadyState {
    /// Head at every node [m].
    pub node_heads: Vec<f64>,
    /// Flow in every pipe [m^3/s].
    pub pipe_flows: Vec<f64>,
    /// For valve nodes: steady flow and head drop at full opening.
    pub valve_steady: Vec<(f64, f64)>,
    /// For check-valve nodes: open or closed.
    pub check_valves: Vec<CheckValveState>,
}

impl InitialCondition {
    /// Resolves and checks the steady state for `network`.
    pub(crate) fn resolve(
        &self,
        network: &Network,
        topo: &Topology,
    ) -> Result<SteadyState, HydraulicsError> {
        let n_nodes = network.nodes().len();
        let bad = |why: String| Err(HydraulicsError::InconsistentInitialState(why));
        match self {
            Self::AtRest { head } => {
                let h = head.value();
                for (i, node) in network.nodes().iter().enumerate() {
                    match node {
                        Node::Reservoir { head: hr } if (hr.value() - h).abs() > HEAD_TOLERANCE => {
                            return bad(format!(
                                "node {i}: reservoir at {hr} cannot be at rest with {head}"
                            ));
                        }
                        Node::Junction { demand } if demand.value() != 0.0 => {
                            return bad(format!(
                                "node {i}: a demand of {demand} means water is flowing"
                            ));
                        }
                        _ => {}
                    }
                }
                Ok(SteadyState {
                    node_heads: vec![h; n_nodes],
                    pipe_flows: vec![0.0; network.pipes().len()],
                    // A valve that passed no flow: its opening has no effect.
                    valve_steady: vec![(0.0, 1.0); n_nodes],
                    check_valves: vec![CheckValveState::Closed; n_nodes],
                })
            }
            Self::PipeFlows(flows) => from_pipe_flows(network, topo, flows),
        }
    }
}

fn from_pipe_flows(
    network: &Network,
    topo: &Topology,
    flows: &[Flow],
) -> Result<SteadyState, HydraulicsError> {
    let bad = |why: String| Err(HydraulicsError::InconsistentInitialState(why));
    let pipes = network.pipes();
    if flows.len() != pipes.len() {
        return bad(format!(
            "{} flows given for {} pipes",
            flows.len(),
            pipes.len()
        ));
    }
    if let Some(q) = flows.iter().find(|q| !q.is_finite()) {
        return bad(format!("pipe flow {q} is not finite"));
    }
    let node_heads = heads_from_reservoirs(network, topo, flows)?;

    // Flows: continuity at every node, and what each device needs.
    let mut valve_steady = vec![(0.0, 1.0); node_heads.len()];
    let mut check_valves = vec![CheckValveState::Closed; node_heads.len()];
    for (i, node) in network.nodes().iter().enumerate() {
        let (mut inflow, mut outflow, mut scale) = (0.0, 0.0, 0.0_f64);
        for &(pid, end) in &topo[i] {
            let q = flows[pid.0].value();
            scale = scale.max(q.abs());
            match end {
                PipeEnd::Downstream => inflow += q,
                PipeEnd::Upstream => outflow += q,
            }
        }
        let tol = FLOW_TOLERANCE * scale.max(1e-6);
        match node {
            Node::Reservoir { .. } => {}
            Node::Junction { demand } => {
                let imbalance = inflow - outflow - demand.value();
                if imbalance.abs() > tol {
                    return bad(format!(
                        "flows do not balance at node {i}: {imbalance:.3e} m^3/s unaccounted"
                    ));
                }
            }
            Node::Valve {
                downstream_head, ..
            } => {
                if inflow < 0.0 {
                    return bad(format!("node {i}: flow into the valve is negative"));
                }
                let drop = node_heads[i] - downstream_head.value();
                if inflow > 0.0 && drop <= 0.0 {
                    return Err(HydraulicsError::NoHeadDropAtValve { head_drop: drop });
                }
                valve_steady[i] = (inflow, if inflow > 0.0 { drop } else { 1.0 });
            }
            Node::CheckValve => {
                if (inflow - outflow).abs() > tol || inflow < 0.0 {
                    return bad(format!(
                        "node {i}: a check valve needs the same forward flow on both sides"
                    ));
                }
                if inflow > 0.0 {
                    check_valves[i] = CheckValveState::Open;
                }
            }
        }
    }

    Ok(SteadyState {
        node_heads,
        pipe_flows: flows.iter().map(|q| q.value()).collect(),
        valve_steady,
        check_valves,
    })
}

/// Heads at every node, walking out from the reservoirs and subtracting
/// each pipe's friction loss. Fails if a loop does not close or a node is
/// not connected to any reservoir.
fn heads_from_reservoirs(
    network: &Network,
    topo: &Topology,
    flows: &[Flow],
) -> Result<Vec<f64>, HydraulicsError> {
    let bad = |why: String| Err(HydraulicsError::InconsistentInitialState(why));
    let pipes = network.pipes();
    let g = network.gravity();
    // Heads: walk out from every reservoir, one pipe at a time.
    let mut heads: Vec<Option<f64>> = vec![None; network.nodes().len()];
    let mut queue = VecDeque::new();
    for (i, node) in network.nodes().iter().enumerate() {
        if let Node::Reservoir { head } = node {
            heads[i] = Some(head.value());
            queue.push_back(i);
        }
    }
    while let Some(n) = queue.pop_front() {
        let h = heads[n].expect("queued nodes have heads");
        for &(pid, end) in &topo[n] {
            let p = &pipes[pid.0];
            let loss = p.pipe.friction_head_loss(flows[pid.0], g).value();
            let (other, h_other) = match end {
                PipeEnd::Upstream => (p.to.0, h - loss), // n is the pipe's `from`
                PipeEnd::Downstream => (p.from.0, h + loss), // n is the pipe's `to`
            };
            match heads[other] {
                Some(existing) if (existing - h_other).abs() > HEAD_TOLERANCE => {
                    return bad(format!(
                        "heads around a loop do not close at node {other}: \
                         {existing:.6} m versus {h_other:.6} m via pipe {}",
                        pid.0
                    ));
                }
                Some(_) => {}
                None => {
                    heads[other] = Some(h_other);
                    queue.push_back(other);
                }
            }
        }
    }
    let mut node_heads = Vec::with_capacity(heads.len());
    for (i, h) in heads.iter().enumerate() {
        match h {
            Some(h) => node_heads.push(*h),
            None => return bad(format!("node {i} is not connected to any reservoir")),
        }
    }
    Ok(node_heads)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Pipe;
    use jk_kernel::{Length, STANDARD_GRAVITY, WaveSpeed};

    fn pipe(f: f64) -> Pipe {
        Pipe::new(
            Length::new(500.0),
            Length::new(0.2),
            WaveSpeed::new(1000.0),
            f,
        )
        .unwrap()
    }

    /// Reservoir feeding a junction that splits into two pipes rejoining
    /// at a second junction with a demand: one loop.
    fn looped(demand: f64) -> Network {
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(80.0),
        });
        let a = net.add_node(Node::Junction { demand: Flow::ZERO });
        let b = net.add_node(Node::Junction {
            demand: Flow::new(demand),
        });
        net.add_pipe(r, a, pipe(0.02)).unwrap();
        net.add_pipe(a, b, pipe(0.02)).unwrap();
        net.add_pipe(a, b, pipe(0.02)).unwrap();
        net
    }

    #[test]
    fn a_symmetric_loop_closes_and_heads_fall_with_friction() {
        let net = looped(0.04);
        let topo = net.topology().unwrap();
        let flows = vec![Flow::new(0.04), Flow::new(0.02), Flow::new(0.02)];
        let s = InitialCondition::PipeFlows(flows)
            .resolve(&net, &topo)
            .unwrap();
        assert!(s.node_heads[1] < 80.0 && s.node_heads[2] < s.node_heads[1]);
    }

    #[test]
    fn an_unbalanced_loop_is_rejected() {
        let net = looped(0.04);
        let topo = net.topology().unwrap();
        // Unequal split through identical pipes: heads cannot close.
        let flows = vec![Flow::new(0.04), Flow::new(0.03), Flow::new(0.01)];
        let err = InitialCondition::PipeFlows(flows).resolve(&net, &topo);
        assert!(matches!(
            err,
            Err(HydraulicsError::InconsistentInitialState(_))
        ));
    }

    #[test]
    fn flows_that_do_not_balance_are_rejected() {
        let net = looped(0.05);
        let topo = net.topology().unwrap();
        let flows = vec![Flow::new(0.04), Flow::new(0.02), Flow::new(0.02)];
        assert!(
            InitialCondition::PipeFlows(flows)
                .resolve(&net, &topo)
                .is_err()
        );
    }

    #[test]
    fn at_rest_needs_equal_reservoirs_and_no_demand() {
        let net = looped(0.0);
        let topo = net.topology().unwrap();
        assert!(
            InitialCondition::AtRest {
                head: Head::new(80.0)
            }
            .resolve(&net, &topo)
            .is_ok()
        );
        assert!(
            InitialCondition::AtRest {
                head: Head::new(70.0)
            }
            .resolve(&net, &topo)
            .is_err()
        );
        let net = looped(0.01);
        let topo = net.topology().unwrap();
        assert!(
            InitialCondition::AtRest {
                head: Head::new(80.0)
            }
            .resolve(&net, &topo)
            .is_err()
        );
    }

    #[test]
    fn a_node_without_a_reservoir_is_rejected() {
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(80.0),
        });
        let a = net.add_node(Node::Junction { demand: Flow::ZERO });
        let b = net.add_node(Node::Junction { demand: Flow::ZERO });
        let c = net.add_node(Node::Junction { demand: Flow::ZERO });
        net.add_pipe(r, a, pipe(0.0)).unwrap();
        net.add_pipe(b, c, pipe(0.0)).unwrap();
        let topo = net.topology().unwrap();
        let flows = vec![Flow::ZERO, Flow::ZERO];
        assert!(
            InitialCondition::PipeFlows(flows)
                .resolve(&net, &topo)
                .is_err()
        );
    }
}
