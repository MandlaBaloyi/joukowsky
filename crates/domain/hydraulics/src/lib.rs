//! Hydraulics bounded context: the physics of transient flow in pipes.
//!
//! Pure domain code (rule R2): no I/O, no clocks, no async. Time is always
//! an input. The theory behind every function is in
//! `docs/02_physics_of_transients/`, and each module names the file it
//! implements.
//!
//! What exists so far (solver milestone S1 in
//! `docs/09_technologies/02_numerics_and_solvers.txt`):
//! - [`joukowsky`]: the closed-form surge relations.
//! - [`Pipe`]: one pipe's geometry, wave speed and friction.
//! - [`ValveClosure`]: how a valve's opening changes with time.
//! - [`ReservoirPipeValve`]: the classic reservoir-pipe-valve system solved
//!   by the method of characteristics (MOC).

mod closure;
mod error;
pub mod joukowsky;
mod pipe;
mod rpv;

pub use closure::ValveClosure;
pub use error::HydraulicsError;
pub use pipe::Pipe;
pub use rpv::{ReservoirPipeValve, Trace};
