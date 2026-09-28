//! Layer 3: WHEN exactly the front arrived (`event_detection.txt`).
//!
//! - [`step_area`]: the step-area method - exact for integrating loggers at
//!   any clock phase.
//!
//! Later: an AIC picker (from seismology) for fronts that are not clean
//! steps, e.g. pump trips and slow valve closures.

pub mod step_area;

pub use step_area::{Onset, OnsetConfig, pick};
