//! Hydraulics bounded context: the physics of transient flow in pipes.
//!
//! Pure domain code (rule R2): no I/O, no clocks, no async. Time is always
//! an input. The theory behind every function is in
//! `docs/02_physics_of_transients/`, and each module names the file it
//! implements.
//!
//! Layout (folders appear as their milestone starts; see
//! `docs/09_technologies/02_numerics_and_solvers.txt` for S1-S9):
//! - [`joukowsky`]: the closed-form surge relations.
//! - [`pipe`]: one pipe's geometry, wave speed and friction.
//! - [`boundary`]: devices at pipe ends (reservoir, valve, ...).
//! - [`moc`]: the method-of-characteristics solver engine and the solvers
//!   built on it.

pub mod boundary;
mod error;
pub mod joukowsky;
pub mod moc;
pub mod pipe;

pub use boundary::closure::ValveClosure;
pub use error::HydraulicsError;
pub use moc::grid::Grid;
pub use moc::single_pipe::ReservoirPipeValve;
pub use moc::trace::Trace;
pub use pipe::Pipe;
