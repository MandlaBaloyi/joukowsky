//! The hydraulic model of a network: nodes, pipes, bursts - shared by the
//! steady-state solver ([`crate::steady`]) and the transient solver
//! ([`crate::moc::network`]).
//!
//! This is the HYDRAULIC model: what the solvers need, nothing more. The
//! network bounded context (asset records, GIS, versions) will translate
//! its richer model into this one.

use jk_kernel::{Acceleration, Area, Flow, Head, Length, Time};

use crate::boundary::control_valve::ValveMode;
use crate::boundary::junction::PipeEnd;
use crate::boundary::pump::Pump;
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
    /// A pump with a discharge non-return valve, joining exactly two pipes:
    /// the suction pipe ends here, the discharge pipe starts here. Runs at
    /// rated speed until `trip`, then runs down under its own inertia.
    Pump {
        /// The pump and its drive.
        pump: Pump,
        /// When the power fails (None: never). Must be after t = 0, since
        /// the steady state assumes the pump is running.
        trip: Option<Time>,
    },
    /// A control valve joining exactly two pipes: the upstream pipe ends
    /// here, the downstream pipe starts here. Pressure-reducing,
    /// pressure-sustaining, flow-control or throttle
    /// ([`crate::boundary::control_valve`]); frozen at its steady opening
    /// in transients.
    ControlValve {
        /// What it does.
        mode: ValveMode,
        /// Fully open: head loss = `open_resistance` Q|Q| [s^2/m^5] (a
        /// throttle valve's loss at its setting).
        open_resistance: f64,
        /// How fast it moves in a transient: the time constant with which
        /// its opening follows the one that would restore its setting
        /// ([`crate::boundary::control_valve::regulating_conductance`]).
        /// `None`: frozen at its steady opening.
        response: Option<Time>,
    },
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

/// A reservoir whose level changes with time - a recorded or planned
/// level history (`docs/02_physics_of_transients/sources_dams_rivers_
/// canals.txt`, step R-1). Levels change over minutes to hours, so within a
/// transient the source is practically fixed; over a longer run the flows
/// follow it.
#[derive(Debug, Clone, PartialEq)]
pub struct SourceLevel {
    /// The reservoir node.
    pub node: NodeId,
    /// (time, head) points, times strictly increasing. Linear between
    /// points; held at the first before it and at the last after it.
    pub history: Vec<(Time, Head)>,
}

impl SourceLevel {
    /// The reservoir's head at time `t`.
    #[must_use]
    pub fn head_at(&self, t: Time) -> Head {
        let h = &self.history;
        let (first, last) = (h[0], h[h.len() - 1]);
        if t <= first.0 {
            return first.1;
        }
        if t >= last.0 {
            return last.1;
        }
        let k = h.partition_point(|&(time, _)| time <= t);
        let ((t0, h0), (t1, h1)) = (h[k - 1], h[k]);
        let f = (t - t0) / (t1 - t0);
        Head::new(h0.value() + f * (h1.value() - h0.value()))
    }
}

/// A storage tank: a reservoir node whose level rises and falls with the
/// water it gains and loses (step R-2 of `docs/02_physics_of_transients/
/// sources_dams_rivers_canals.txt`). Used by the extended-period run
/// ([`crate::steady::extended`]); in a transient, which lasts seconds, a
/// tank's level is practically fixed and it acts as a reservoir.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TankStorage {
    /// The reservoir node that is the tank; its head is the starting level.
    pub node: NodeId,
    /// Plan area of the water surface (constant with height).
    pub area: Area,
    /// Lowest usable level: below it the tank supplies nothing.
    pub floor: Head,
    /// Highest level: above it the tank overflows.
    pub ceiling: Head,
}

/// A pipe network for transient simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct Network {
    gravity: Acceleration,
    nodes: Vec<Node>,
    pipes: Vec<NetworkPipe>,
    bursts: Vec<Burst>,
    levels: Vec<SourceLevel>,
    tanks: Vec<TankStorage>,
    /// Ground elevation of each node [m] (0 unless set): pressure head =
    /// head - elevation, for pressure-driven demand.
    elevations: Vec<f64>,
    /// Gauge pressure head at which water boils, if transients may
    /// cavitate (S7).
    vapour_floor: Option<f64>,
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
            levels: Vec::new(),
            tanks: Vec::new(),
            elevations: Vec::new(),
            vapour_floor: None,
        }
    }

    /// Lets water boil in transients: wherever the pressure head would fall
    /// below `floor` (the vapour pressure as a gauge head - about -10 m at
    /// the coast, see [`crate::vapour::vapour_floor`]) a vapour cavity opens
    /// instead, and closes when the water returns (DVCM, `docs/
    /// 02_physics_of_transients/column_separation_and_intrusion.txt`
    /// section 5). Pressure heads are measured from each node's ground
    /// elevation, the pipe running straight between its nodes. `None` (the
    /// default): no floor - linear elastic theory, heads may go anywhere.
    ///
    /// # Errors
    /// A floor that is not finite or not below 0 (water boils below
    /// atmospheric pressure only).
    pub fn set_vapour_floor(&mut self, floor: Option<Head>) -> Result<(), HydraulicsError> {
        if let Some(f) = floor
            && !(f.value().is_finite() && f.value() < 0.0)
        {
            return Err(HydraulicsError::invalid(
                "vapour floor",
                f.value(),
                "finite and below 0 (gauge)",
            ));
        }
        self.vapour_floor = floor.map(Head::value);
        Ok(())
    }

    /// The vapour floor, if set.
    #[must_use]
    pub fn vapour_floor(&self) -> Option<Head> {
        self.vapour_floor.map(Head::new)
    }

    /// Sets node `node`'s ground elevation (used where a pressure matters:
    /// pressure-driven demand).
    ///
    /// # Errors
    /// An unknown node or an elevation that is not finite.
    pub fn set_elevation(
        &mut self,
        node: NodeId,
        elevation: Length,
    ) -> Result<(), HydraulicsError> {
        finite("elevation", elevation.value())?;
        let slot = self.elevations.get_mut(node.0).ok_or_else(|| {
            HydraulicsError::InvalidNetwork(format!("node {} does not exist", node.0))
        })?;
        *slot = elevation.value();
        Ok(())
    }

    /// Node `node`'s ground elevation (0 unless set).
    #[must_use]
    pub fn elevation(&self, node: NodeId) -> Length {
        Length::new(self.elevations.get(node.0).copied().unwrap_or(0.0))
    }

    /// Adds a node and returns its id.
    pub fn add_node(&mut self, node: Node) -> NodeId {
        self.elevations.push(0.0);
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

    /// Gives a reservoir a level history: from now on its head follows
    /// `level` instead of the fixed head of its node.
    ///
    /// # Errors
    /// If the node is not a reservoir or already has a history, the history
    /// is empty, or its times are not finite and strictly increasing or its
    /// heads not finite.
    pub fn add_level(&mut self, level: SourceLevel) -> Result<(), HydraulicsError> {
        if !matches!(self.nodes.get(level.node.0), Some(Node::Reservoir { .. })) {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "a level history must be at a reservoir node (node {})",
                level.node.0
            )));
        }
        if self.levels.iter().any(|l| l.node == level.node) {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "node {} already has a level history",
                level.node.0
            )));
        }
        if level.history.is_empty() {
            return Err(HydraulicsError::InvalidNetwork(
                "a level history needs at least one point".into(),
            ));
        }
        for (i, &(t, h)) in level.history.iter().enumerate() {
            finite("level time", t.value())?;
            finite("level head", h.value())?;
            if i > 0 && t <= level.history[i - 1].0 {
                return Err(HydraulicsError::invalid(
                    "level time",
                    t.value(),
                    "strictly after the previous point",
                ));
            }
        }
        self.levels.push(level);
        Ok(())
    }

    /// The level histories given to reservoirs.
    #[must_use]
    pub fn levels(&self) -> &[SourceLevel] {
        &self.levels
    }

    /// Makes a reservoir a storage tank: in an extended-period run its level
    /// follows the water it gains and loses.
    ///
    /// # Errors
    /// If the node is not a reservoir, already a tank or given a level
    /// history, the area is not positive, or the floor is not below the
    /// ceiling with the starting level between them.
    pub fn add_tank(&mut self, tank: TankStorage) -> Result<(), HydraulicsError> {
        let Some(Node::Reservoir { head }) = self.nodes.get(tank.node.0) else {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "a tank must be a reservoir node (node {})",
                tank.node.0
            )));
        };
        if self.tanks.iter().any(|t| t.node == tank.node)
            || self.levels.iter().any(|l| l.node == tank.node)
        {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "node {} is already a tank or follows a level history",
                tank.node.0
            )));
        }
        positive("tank area", tank.area.value())?;
        finite("tank floor", tank.floor.value())?;
        finite("tank ceiling", tank.ceiling.value())?;
        if !(tank.floor < tank.ceiling && tank.floor <= *head && *head <= tank.ceiling) {
            return Err(HydraulicsError::invalid(
                "tank level",
                head.value(),
                "floor < ceiling, starting level between them",
            ));
        }
        self.tanks.push(tank);
        Ok(())
    }

    /// The storage tanks.
    #[must_use]
    pub fn tanks(&self) -> &[TankStorage] {
        &self.tanks
    }

    /// Replaces node `n` (the extended-period run re-levels tanks, closes
    /// empty ones and scales demands on its own copy of the network).
    pub(crate) fn set_node(&mut self, n: usize, node: Node) {
        self.nodes[n] = node;
    }

    /// Drops every level history, the heads having been written into the
    /// reservoir nodes (the extended-period run's copy).
    pub(crate) fn clear_levels(&mut self) {
        self.levels.clear();
    }

    /// The head of reservoir node `node` (fixed head `fixed`) at time `t`:
    /// its level history if it has one.
    pub(crate) fn source_head(&self, node: usize, fixed: Head, t: Time) -> f64 {
        self.levels
            .iter()
            .find(|l| l.node.0 == node)
            .map_or(fixed.value(), |l| l.head_at(t).value())
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

    /// The id of the pipe at `index` in [`Self::pipes`], if it exists.
    #[must_use]
    pub fn pipe_id(&self, index: usize) -> Option<PipeId> {
        (index < self.pipes.len()).then_some(PipeId(index))
    }

    /// The id of the node at `index` in [`Self::nodes`], if it exists.
    #[must_use]
    pub fn node_id(&self, index: usize) -> Option<NodeId> {
        (index < self.nodes.len()).then_some(NodeId(index))
    }

    /// All scheduled bursts.
    #[must_use]
    pub fn bursts(&self) -> &[Burst] {
        &self.bursts
    }

    /// Checks the rules every node must satisfy and returns, for each
    /// node, the pipes touching it.
    #[allow(clippy::too_many_lines)] // one check per kind of node
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
                Node::ControlValve {
                    mode,
                    open_resistance,
                    response,
                } => {
                    if let Some(tau) = response {
                        positive("valve response time", tau.value())?;
                    }
                    match mode {
                        ValveMode::Prv { setting } | ValveMode::Psv { setting } => {
                            finite("valve setting", setting.value())?;
                        }
                        ValveMode::Fcv { flow } => {
                            if !(flow.value().is_finite() && flow.value() >= 0.0) {
                                return Err(HydraulicsError::invalid(
                                    "FCV flow",
                                    flow.value(),
                                    "finite and >= 0",
                                ));
                            }
                        }
                        ValveMode::Tcv => {}
                    }
                    if !(open_resistance.is_finite() && *open_resistance >= 0.0) {
                        return Err(HydraulicsError::invalid(
                            "valve open_resistance",
                            *open_resistance,
                            "finite and >= 0",
                        ));
                    }
                    if !(ending == 1 && starting == 1) {
                        return fail(
                            "a control valve must join one pipe ending and one starting here",
                        );
                    }
                }
                Node::Pump { pump, trip } => {
                    pump.validate()?;
                    if let Some(t) = trip {
                        positive("pump trip time", t.value())?;
                    }
                    if !(ending == 1 && starting == 1) {
                        return fail(
                            "a pump must join its suction pipe (ending here) and its \
                             discharge pipe (starting here)",
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
