//! The method of characteristics (MOC): the transient solver engine.
//!
//! Implements `docs/02_physics_of_transients/governing_equations_and_moc.txt`.
//! The pieces every MOC solver shares live here once:
//!   - [`grid`]: reaches, time step, step counts (Courant number 1).
//!   - [`characteristics`]: the C+ / C- compatibility equations.
//!   - [`trace`]: recorded head and flow histories.
//!
//! Solvers built from them:
//!   - [`single_pipe`]: reservoir - pipe - valve (milestone S1).
//!   - [`network`]: many connected pipes (milestone S3).

pub mod characteristics;
pub mod grid;
pub mod network;
pub mod single_pipe;
pub mod trace;
