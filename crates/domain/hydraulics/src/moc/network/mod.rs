//! Network MOC: many pipes joined at nodes, solved together (milestone S3).
//!
//! `docs/02_physics_of_transients/governing_equations_and_moc.txt` sections
//! 2-3. It runs on the hydraulic model in [`crate::model`]. The pieces:
//!   - [`grid`]: one shared time step; each pipe gets a whole number of
//!     reaches, which forces small, recorded wave-speed adjustments.
//!   - [`initial`]: the steady state the transient starts from.
//!   - [`solver`]: the time loop.

pub mod grid;
pub mod initial;
pub mod solver;

pub use grid::{GridOptions, NetworkGrid};
pub use initial::InitialCondition;
pub use solver::{NetworkRun, Probe};
