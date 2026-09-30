//! Steady state: the everyday flows and pressures of a network
//! (milestone S4).
//!
//! Given reservoir levels, consumer demands and valve settings, find the
//! flow in every pipe and the head at every node. This is the state every
//! transient starts from ([`crate::InitialCondition::Steady`]) and, on its
//! own, the baseline the engine compares measurements against.
//!
//! Method: the Global Gradient Algorithm of Todini & Pilati (1988), the
//! algorithm inside EPANET, solved with our own sparse envelope Cholesky
//! with reverse Cuthill-McKee ordering ([`jk_kernel::sparse`]; dense until
//! 2026-09-30, limited to ~2000 junctions)
//! (decision TD-011: our own solver; EPANET-based tools are cross-checks
//! only). See [`gga`] for the equations.
//!
//! Friction is the same quasi-steady Darcy-Weisbach model (constant f per
//! pipe) the transient solver uses, so steady and transient runs agree.

pub mod controls;
pub mod extended;
pub mod gga;

pub(crate) use gga::solve;

use jk_kernel::{Flow, Head};

use crate::HydraulicsError;
use crate::error::positive;
use crate::model::{Network, Node, NodeId, PipeId};

/// Settings for the steady solve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteadyOptions {
    /// Most Newton rounds before giving up.
    pub max_iterations: u32,
    /// Converged when no pipe flow changes by more than this fraction of
    /// the largest flow (or of 1 l/s, if flows are smaller).
    pub tolerance: f64,
}

impl Default for SteadyOptions {
    fn default() -> Self {
        Self {
            max_iterations: 200,
            tolerance: 1e-10,
        }
    }
}

/// The steady state found, with evidence of how well it satisfies physics.
#[derive(Debug, Clone, PartialEq)]
pub struct SteadySolution {
    /// Head at every node, indexed like [`Network::nodes`]. At a shut check
    /// valve, the upstream side's head.
    pub node_heads: Vec<Head>,
    /// Flow in every pipe, indexed like [`Network::pipes`], positive from
    /// the pipe's `from` node to its `to` node.
    pub pipe_flows: Vec<Flow>,
    /// Newton rounds used (in the final check-valve pass).
    pub iterations: u32,
    /// Largest mismatch between a pipe's friction loss and its head drop.
    pub max_energy_residual: Head,
    /// Largest flow imbalance at any junction.
    pub max_continuity_residual: Flow,
    /// Check valves that are shut in the steady state.
    pub closed_check_valves: Vec<NodeId>,
    /// Pumps that deliver nothing: stopped ([`Shut`]), or unable to
    /// overcome the head against them (their non-return valve shuts).
    pub idle_pumps: Vec<NodeId>,
    /// Head each pump adds (discharge minus suction head).
    pub pump_lifts: Vec<(NodeId, Head)>,
    /// What each control valve is doing.
    pub valves: Vec<(NodeId, crate::boundary::control_valve::ValveState)>,
}

/// Links out of service for a steady solve: pipes closed (a shut
/// isolation valve, a float valve on a full tank) and pumps stopped.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Shut {
    /// Pipes carrying nothing.
    pub pipes: Vec<PipeId>,
    /// Pump nodes not running (no power, or switched off).
    pub pumps: Vec<NodeId>,
}

impl Shut {
    /// Nothing shut.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pipes.is_empty() && self.pumps.is_empty()
    }
}

impl Network {
    /// Solves for the steady state.
    ///
    /// # Errors
    /// Invalid topology or options, a valve without a flow coefficient, a
    /// part of the network that reaches no reservoir, a nodal matrix too
    /// large for memory, or no convergence.
    pub fn steady_state(&self, options: &SteadyOptions) -> Result<SteadySolution, HydraulicsError> {
        validate(options)?;
        let topo = self.topology()?;
        solve(self, &topo, options).map(|(solution, _)| solution)
    }

    /// Solves for the steady state with some links out of service. Parts of
    /// the network cut off with no demand get NaN heads (unknown: the
    /// trapped water keeps whatever pressure it had).
    ///
    /// # Errors
    /// As [`Network::steady_state`]; also a pipe or pump in `shut` that
    /// does not exist, and a part cut off from every source that draws
    /// water (supply is lost there).
    pub fn steady_state_shut(
        &self,
        options: &SteadyOptions,
        shut: &Shut,
    ) -> Result<SteadySolution, HydraulicsError> {
        validate(options)?;
        let topo = self.topology()?;
        self.check_shut(shut)?;
        gga::solve_shut(self, &topo, options, shut).map(|(solution, _)| solution)
    }

    /// Checks that every pipe and pump in `shut` exists.
    pub(crate) fn check_shut(&self, shut: &Shut) -> Result<(), HydraulicsError> {
        if let Some(p) = shut.pipes.iter().find(|p| p.0 >= self.pipes().len()) {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "pipe {} to shut does not exist",
                p.0
            )));
        }
        if let Some(n) = shut
            .pumps
            .iter()
            .find(|n| !matches!(self.nodes().get(n.0), Some(Node::Pump { .. })))
        {
            return Err(HydraulicsError::InvalidNetwork(format!(
                "node {} to stop is not a pump",
                n.0
            )));
        }
        Ok(())
    }
}

pub(crate) fn validate(options: &SteadyOptions) -> Result<(), HydraulicsError> {
    if options.max_iterations == 0 {
        return Err(HydraulicsError::invalid("max_iterations", 0.0, ">= 1"));
    }
    positive("tolerance", options.tolerance)
}
