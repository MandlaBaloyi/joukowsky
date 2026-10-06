//! Localisation tests: where on the network a transient came from.
//!
//! Code under test: `crates/domain/localisation`. Grouped by rung:
//! - [`analytical`]: exact arrival times from geometry - a straight main,
//!   and a branched network where two sensors CANNOT decide and a third can.
//! - [`invariants`]: the 90% region really holds the truth about 90% of the
//!   time, even with uncertain wave speeds (calibration, principle P3).
//! - [`simulation`]: rung 3 end to end - hydraulics -> loggers -> detection
//!   -> localisation on the branched network.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only localisation`

mod analytical;
mod datasets;
mod invariants;
mod simulation;

pub use jk_tests::estate::{A, Estate, estate};

/// Exact arrival times at S1, S2, S3 for a burst `d` metres from T along
/// T-S3, emitted at `t0`.
#[must_use]
pub fn arrivals_on_branch(d: f64, t0: f64) -> [f64; 3] {
    [
        t0 + (d + 300.0) / A,
        t0 + (d + 500.0) / A,
        t0 + (600.0 - d) / A,
    ]
}
