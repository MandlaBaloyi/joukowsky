//! Steady state: the everyday flows and pressures of a network
//! (milestone S4).
//!
//! Given reservoir levels, consumer demands and valve settings, find the
//! flow in every pipe and the head at every node. This is the state every
//! transient starts from ([`crate::InitialCondition::Steady`]) and, on its
//! own, the baseline the engine compares measurements against.
//!
//! Method: the Global Gradient Algorithm of Todini & Pilati (1988), the
//! algorithm inside EPANET, solved with our own dense Cholesky factorisation
//! (decision TD-011: our own solver; EPANET-based tools are cross-checks
//! only). See [`gga`] for the equations.
//!
//! Friction is the same quasi-steady Darcy-Weisbach model (constant f per
//! pipe) the transient solver uses, so steady and transient runs agree.

mod dense;
pub mod gga;

pub(crate) use gga::solve;

use jk_kernel::{Flow, Head};

use crate::HydraulicsError;
use crate::error::positive;
use crate::model::{Network, NodeId};

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
}

impl Network {
    /// Solves for the steady state.
    ///
    /// # Errors
    /// Invalid topology or options, a valve without a flow coefficient, a
    /// part of the network that reaches no reservoir, too many nodes for
    /// the dense solver, or no convergence.
    pub fn steady_state(&self, options: &SteadyOptions) -> Result<SteadySolution, HydraulicsError> {
        validate(options)?;
        let topo = self.topology()?;
        solve(self, &topo, options).map(|(solution, _)| solution)
    }
}

pub(crate) fn validate(options: &SteadyOptions) -> Result<(), HydraulicsError> {
    if options.max_iterations == 0 {
        return Err(HydraulicsError::invalid("max_iterations", 0.0, ">= 1"));
    }
    positive("tolerance", options.tolerance)
}
