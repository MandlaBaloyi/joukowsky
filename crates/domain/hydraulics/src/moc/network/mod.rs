//! Network MOC: many pipes joined at nodes, solved together (milestone S3).
//!
//! `docs/02_physics_of_transients/governing_equations_and_moc.txt` sections
//! 2-3. The pieces:
//!   - [`model`]: the hydraulic model - nodes, pipes, bursts - and its rules.
//!   - [`grid`]: one shared time step; each pipe gets a whole number of
//!     reaches, which forces small, recorded wave-speed adjustments.
//!   - [`initial`]: the steady state the transient starts from.
//!   - [`solver`]: the time loop.
//!
//! This is the HYDRAULIC model: what the solver needs, nothing more. The
//! network bounded context (asset records, GIS, versions) will translate
//! its richer model into this one.

pub mod grid;
pub mod initial;
pub mod model;
pub mod solver;

pub use grid::{GridOptions, NetworkGrid};
pub use initial::InitialCondition;
pub use model::{Burst, BurstOpening, Network, Node, NodeId, PipeId};
pub use solver::{NetworkRun, Probe};
