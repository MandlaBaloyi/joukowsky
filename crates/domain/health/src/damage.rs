//! Fatigue damage from counted cycles: Miner's rule on a power-law S-N
//! curve, and the relative measure that needs no S-N data at all
//! (`docs/02_physics_of_transients/pressure_leakage_and_fatigue.txt`
//! section 2; `docs/09_technologies/11_frontier_what_we_must_invent.txt`
//! F7).
//!
//! ```text
//!   S-N curve (Basquin):  N(S) = N_ref (S_ref / S)^m   cycles to failure at range S
//!   Miner:                D = sum n_i / N(S_i)          failure expected near D = 1
//!   equivalent cycles:    n_eq = sum n_i (S_i / S_ref)^m
//! ```
//!
//! The pressure range stands in for the wall's stress range (hoop stress
//! = p D / 2e, the same factor for every cycle of one pipe). S-N data for
//! AGED South African pipes is not known, so absolute damage is a guess;
//! but `n_eq` ranks pipes with only the exponent m: "this pipe has taken the
//! equivalent of 1 200 swings of 50 m, that one 90". Mean pressure (which
//! matters for plastics) is not yet used.

use crate::rainflow::Cycle;

/// A material's S-N curve as a power law in the pressure range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SnCurve {
    /// A pressure range [m] ...
    pub reference_range: f64,
    /// ... and the cycles of it the pipe survives.
    pub reference_cycles: f64,
    /// Slope m: halving the range multiplies the life by 2^m.
    pub exponent: f64,
}

impl SnCurve {
    /// Cycles to failure at pressure range `range`.
    #[must_use]
    pub fn cycles_to_failure(&self, range: f64) -> f64 {
        self.reference_cycles * (self.reference_range / range).powf(self.exponent)
    }
}

/// Miner's damage of `cycles` on `curve` (failure expected near 1).
#[must_use]
pub fn miner(cycles: &[Cycle], curve: &SnCurve) -> f64 {
    cycles
        .iter()
        .filter(|c| c.range > 0.0)
        .map(|c| c.count / curve.cycles_to_failure(c.range))
        .sum()
}

/// The cycles of `reference_range` that would do the same damage as
/// `cycles`, for S-N exponent `exponent`.
#[must_use]
pub fn equivalent_cycles(cycles: &[Cycle], reference_range: f64, exponent: f64) -> f64 {
    cycles
        .iter()
        .map(|c| c.count * (c.range / reference_range).powf(exponent))
        .sum()
}
