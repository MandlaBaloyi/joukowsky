//! Detection tests: finding transient events in pressure signals.
//!
//! Code under test: `crates/domain/detection`. Grouped by validation rung
//! (`docs/07_validation/validation_strategy.txt`):
//!
//! - [`analytical`]: behaviour with a known answer - exact arrival times,
//!   predictable delays, no false alarms on pure noise, drift ignored.
//! - [`simulation`]: rung 3 - the hydraulics simulator makes a burst,
//!   synthetic 128 S/s loggers record it, the detector must find and time
//!   it well enough to LOCATE it (study V-S1).
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only detection`

mod analytical;
mod simulation;

use jk_detection::{Detection, Detector, DetectorConfig, Series};
use jk_kernel::Time;

/// Runs the default detector on samples taken `rate` times a second,
/// the first at time `start`.
///
/// # Panics
/// If detection fails (bad input in a test is a test bug).
#[must_use]
pub fn detect(samples: &[f64], rate: f64, start: f64) -> Detection {
    detect_with(samples, rate, start, DetectorConfig::default())
}

/// As [`detect`], with explicit settings.
///
/// # Panics
/// If detection fails.
#[must_use]
pub fn detect_with(samples: &[f64], rate: f64, start: f64, config: DetectorConfig) -> Detection {
    Detector::new(config)
        .expect("valid config")
        .run(&Series {
            samples,
            start: Time::new(start),
            interval: Time::new(1.0 / rate),
        })
        .expect("detection runs")
}
