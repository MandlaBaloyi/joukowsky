//! The hydraulic model of a network: nodes, pipes, bursts - shared by the
//! steady-state solver ([`crate::steady`]) and the transient solver
//! ([`crate::moc::network`]).
//!
//! This is the HYDRAULIC model: what the solvers need, nothing more. The
//! network bounded context (asset records, GIS, versions) will translate
//! its richer model into this one.

use jk_kernel::{Acceleration, Flow, Head, Time};

use crate::boundary::junction::PipeEnd;
use crate::error::{finite, positive};
use crate::{HydraulicsError, Pipe, ValveClosure};

/// Identifies a node in a [`Network`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeId(pub(crate) usize);

impl NodeId {
    /// Position of the node in [`Network::nodes`].
    #[must_use]
    pub fn index(self) -> usize {
        self.0
    }
}

/// Identifies a pipe in a [`Network`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct PipeId(pub(crate) usize);

impl PipeId {
    /// Position of the pipe in [`Network::pipes`].
    #[must_use]
    pub fn index(self) -> usize {
        self.0
    }
}

/// What sits at a node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Node {
    /// Fixed head, whatever the flow. Any number of pipes.
    Reservoir {
        /// The reservoir's head.
        head: Head,
    },
    /// Pipes meeting, with a fixed outflow (consumption); 0 for a plain
    /// junction. One pipe and no demand is a dead end.
    Junction {
        /// Flow leaving the network here.
        demand: Flow,
    },
    /// A valve at the downstream end of exactly one pipe, discharging to a
    /// fixed head.
    Valve {
        /// Head below the valve (0 for discharge to atmosphere).
        downstream_head: Head,
        /// How the valve moves from t = 0.
        closure: ValveClosure,
        /// Flow coefficient at the initial opening, Kv in
        /// Q = Kv sqrt(H - `H_down`) [m^2.5/s]; 0 = shut. Needed by the
        /// steady-state solver. With given pipe flows it may be left out
        /// (None) - if given, the flows are checked against it.
        flow_coefficient: Option<f64>,
    },
    /// An ideal check valve joining exactly two pipes: one ending here
    /// (upstream side) and one starting here (downstream side).
    CheckValve,
}

/// A pipe placed between two nodes. Positive flow runs `from` -> `to`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NetworkPipe {
    /// The pipe.
    pub pipe: Pipe,
    /// Upstream node.
    pub from: NodeId,
    /// Downstream node.
    pub to: NodeId,
}

/// How a burst's opening grows once it starts.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BurstOpening {
    /// Fully open at the first time step at or after the start time.
    Instantaneous,
    /// Opening grows linearly to full over `duration`.
    Linear {
        /// Time to open fully.
        duration: Time,
    },
}

/// A burst (or new leak) at a junction node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Burst {
    /// The junction where the pipe fails.
    pub node: NodeId,
    /// Final orifice coefficient k = Cd A sqrt(2 g), see
    /// [`crate::boundary::leak::coefficient`].
    pub coefficient: f64,
    /// Head outside the opening (ground or trench water level).
    pub outside_head: Head,
    /// When the pipe fails.
    pub start: Time,
    /// How the opening grows.
    pub opening: BurstOpening,
}

impl Burst {
    /// Orifice coefficient at time `t` (0 before the burst).
    #[must_use]
    pub fn coefficient_at(&self, t: Time) -> f64 {
        let since = (t - self.start).value();
        if since < 0.0 {
            return 0.0;
        }
        match self.opening {
            BurstOpening::Instantaneous => self.coefficient,
            BurstOpening::Linear { duration } => {
                self.coefficient * (since / duration.value()).min(1.0)
            }
        }
    }
}

/// A pipe network for transient simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct Network {
    gravity: Acceleration,
    nodes: Vec<Node>,
    pipes: Vec<NetworkPipe>,
    bursts: Vec<Burst>,
}

/// For each node, the pipes that touch it and which end of each.
pub(crate) type Topology = Vec<Vec<(PipeId, PipeEnd)>>;

impl Network {
    /// An empty network.
    #[must_use]
    pub fn new(gravity: Acceleration) -> Self {
        Self {
            gravity,
            nodes: Vec::new(),
            pipes: Vec::new(),
            bursts: Vec::new(),
        }
    }

    /// Adds a node and returns its id.
    pub fn add_node(&mut self, node: Node) -> NodeId {
        self.nodes.push(node);
        NodeId(self.nodes.len() - 1)
    }

    /// Adds a pipe from `from` to `to` and returns its id.
    ///
    /// # Errors
    /// If either node does not exist, or `from == to`.
    pub fn add_pipe(
        &mut self,
        from: NodeId,
        to: NodeId,
        pipe: Pipe,
    ) -> Result<PipeId, HydraulicsError> {
        for id in [from, to] {
            if id.0 >= self.nodes.len() {
                return Err(HydraulicsError::InvalidNetwork(format!(
                    "pipe refers to node {} which does not exist",
                    id.0
                )));
            }
        }
        if from == to {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "pipe starts and ends at node {}",
                from.0
            )));
        }
        self.pipes.push(NetworkPipe { pipe, from, to });
        Ok(PipeId(self.pipes.len() - 1))
    }

    /// Schedules a burst.
    ///
    /// # Errors
    /// If the node is not a junction, already has a burst, or the burst's
    /// numbers are not physical.
    pub fn add_burst(&mut self, burst: Burst) -> Result<(), HydraulicsError> {
        match self.nodes.get(burst.node.0) {
            Some(Node::Junction { .. }) => {}
            _ => {
                return Err(HydraulicsError::InvalidNetwork(format!(
                    "a burst must be at a junction node (node {})",
                    burst.node.0
                )));
            }
        }
        if self.bursts.iter().any(|b| b.node == burst.node) {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "node {} already has a burst",
                burst.node.0
            )));
        }
        if !(burst.coefficient.is_finite() && burst.coefficient >= 0.0) {
            return Err(HydraulicsError::invalid(
                "burst coefficient",
                burst.coefficient,
                "finite and >= 0",
            ));
        }
        finite("burst outside_head", burst.outside_head.value())?;
        finite("burst start", burst.start.value())?;
        if let BurstOpening::Linear { duration } = burst.opening {
            positive("burst opening duration", duration.value())?;
        }
        self.bursts.push(burst);
        Ok(())
    }

    /// Gravitational acceleration.
    #[must_use]
    pub fn gravity(&self) -> Acceleration {
        self.gravity
    }

    /// All nodes, indexed by [`NodeId::index`].
    #[must_use]
    pub fn nodes(&self) -> &[Node] {
        &self.nodes
    }

    /// All pipes, indexed by [`PipeId::index`].
    #[must_use]
    pub fn pipes(&self) -> &[NetworkPipe] {
        &self.pipes
    }

    /// All scheduled bursts.
    #[must_use]
    pub fn bursts(&self) -> &[Burst] {
        &self.bursts
    }

    /// Checks the rules every node must satisfy and returns, for each
    /// node, the pipes touching it.
    pub(crate) fn topology(&self) -> Result<Topology, HydraulicsError> {
        positive("gravity", self.gravity.value())?;
        if self.pipes.is_empty() {
            return Err(HydraulicsError::InvalidNetwork("no pipes".into()));
        }
        let mut topo: Topology = vec![Vec::new(); self.nodes.len()];
        for (i, p) in self.pipes.iter().enumerate() {
            topo[p.from.0].push((PipeId(i), PipeEnd::Upstream));
            topo[p.to.0].push((PipeId(i), PipeEnd::Downstream));
        }
        for (n, (node, touching)) in self.nodes.iter().zip(&topo).enumerate() {
            let ending = touching
                .iter()
                .filter(|(_, e)| *e == PipeEnd::Downstream)
                .count();
            let starting = touching.len() - ending;
            let fail = |why: &str| Err(HydraulicsError::InvalidNetwork(format!("node {n}: {why}")));
            match node {
                Node::Reservoir { head } => {
                    finite("reservoir head", head.value())?;
                    if touching.is_empty() {
                        return fail("reservoir has no pipes");
                    }
                }
                Node::Junction { demand } => {
                    finite("junction demand", demand.value())?;
                    if touching.is_empty() {
                        return fail("junction has no pipes");
                    }
                }
                Node::Valve {
                    downstream_head,
                    closure,
                    flow_coefficient,
                } => {
                    finite("valve downstream_head", downstream_head.value())?;
                    closure.validate()?;
                    if let Some(kv) = flow_coefficient
                        && !(kv.is_finite() && *kv >= 0.0)
                    {
                        return Err(HydraulicsError::invalid(
                            "valve flow_coefficient",
                            *kv,
                            "finite and >= 0",
                        ));
                    }
                    if !(ending == 1 && starting == 0) {
                        return fail("a valve must end exactly one pipe and start none");
                    }
                }
                Node::CheckValve => {
                    if !(ending == 1 && starting == 1) {
                        return fail(
                            "a check valve must join one pipe ending and one starting here",
                        );
                    }
                }
            }
        }
        Ok(topo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::{Length, STANDARD_GRAVITY, WaveSpeed};

    fn pipe() -> Pipe {
        Pipe::new(
            Length::new(100.0),
            Length::new(0.15),
            WaveSpeed::new(1000.0),
            0.02,
        )
        .unwrap()
    }

    #[test]
    fn rejects_broken_topologies() {
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        assert!(net.add_pipe(r, r, pipe()).is_err(), "self-loop");
        assert!(net.add_pipe(r, NodeId(9), pipe()).is_err(), "missing node");

        let v = net.add_node(Node::Valve {
            downstream_head: Head::ZERO,
            closure: ValveClosure::Open,
            flow_coefficient: None,
        });
        net.add_pipe(v, r, pipe()).unwrap(); // valve STARTS a pipe: not allowed
        assert!(net.topology().is_err());
    }

    #[test]
    fn bursts_only_at_junctions_and_one_per_node() {
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        let j = net.add_node(Node::Junction { demand: Flow::ZERO });
        net.add_pipe(r, j, pipe()).unwrap();
        let burst = |node| Burst {
            node,
            coefficient: 1e-4,
            outside_head: Head::ZERO,
            start: Time::new(1.0),
            opening: BurstOpening::Instantaneous,
        };
        assert!(net.add_burst(burst(r)).is_err());
        assert!(net.add_burst(burst(j)).is_ok());
        assert!(net.add_burst(burst(j)).is_err());
    }

    #[test]
    fn burst_opening_schedules() {
        let b = Burst {
            node: NodeId(0),
            coefficient: 2.0,
            outside_head: Head::ZERO,
            start: Time::new(1.0),
            opening: BurstOpening::Linear {
                duration: Time::new(0.5),
            },
        };
        assert!(b.coefficient_at(Time::new(0.9)).abs() < f64::EPSILON);
        assert!((b.coefficient_at(Time::new(1.25)) - 1.0).abs() < 1e-12);
        assert!((b.coefficient_at(Time::new(3.0)) - 2.0).abs() < f64::EPSILON);
    }
}
