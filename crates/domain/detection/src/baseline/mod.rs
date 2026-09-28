//! Layer 1: the slowly moving "normal" level a transient is measured
//! against (`event_detection.txt`).
//!
//! - [`brown`]: Brown's double exponential smoothing - follows demand drift
//!   (level and slope) with no lag on a steady trend.
//!
//! Later: a small Kalman filter that also carries the level's uncertainty.

pub mod brown;

pub use brown::Baseline;
