//! Layer 2: deciding that something changed (`event_detection.txt`).
//!
//! - [`cusum`]: two-sided CUSUM (Page 1954) - cheap, allocation-free,
//!   edge-ready.
//!
//! Later, benchmarked against it by ROC curves (docs 09 04 section 3): GLR
//! step test, multi-scale wavelet energy, Bayesian online change-point
//! detection (which also gives a confidence).

pub mod cusum;

pub use cusum::{Alarm, Cusum, CusumConfig, Direction};
