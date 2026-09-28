//! How much a quiet sensor wiggles - the unit every threshold is set in.
//!
//! - [`robust`]: median-based noise from sample-to-sample differences,
//!   proof against drift and the odd spike.
//!
//! Later: heavy-tailed noise models, and per-sensor noise profiles learned
//! over weeks (time of day, pump states).

pub mod robust;

pub use robust::{median, sigma_from_differences};
