//! Health bounded context: what the pressure history does to the pipes
//! (`docs/02_physics_of_transients/pressure_leakage_and_fatigue.txt`
//! sections 2-3; `docs/06_engine_architecture/project_structure_and_ddd.txt`).
//!
//! Pure: pressure records in, loading measures out; no clock reads.
//!
//! - [`rainflow`]: a record into load cycles (ASTM E1049), live or whole.
//! - [`damage`]: Miner's rule, and equivalent cycles - a ranking of pipes
//!   by the fatigue loading they took, without S-N data for aged pipes.
//! - [`spectrum`]: a running histogram of cycles by range.

pub mod damage;
pub mod rainflow;
pub mod spectrum;

mod error;

pub use damage::{SnCurve, equivalent_cycles, miner};
pub use error::HealthError;
pub use rainflow::{Counter, Cycle, rainflow};
pub use spectrum::Spectrum;
