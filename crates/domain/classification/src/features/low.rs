//! How low the pressure went after the front
//! (`docs/02_physics_of_transients/column_separation_and_intrusion.txt`
//! sections 2-3).
//!
//! Two dangers live below zero, and neither is a class of its own - a pump
//! trip, a valve slam or a burst can each bring them:
//! - NEGATIVE pressure (below atmospheric): water outside the pipe - trench
//!   water, sewage - can be drawn in through leaks and joints. A water
//!   quality event; its exposure is duration x depth.
//! - the VAPOUR FLOOR: the water boils and the column separates; when the
//!   columns meet again the collapse can drive the pressure to about twice
//!   the usual surge (FN-035).
//!
//! The logger reads gauge pressure head at the sensor, so the floor is a
//! gauge head too: about -10.1 m at the coast, -8.2 m at 1700 m. A logger
//! whose range stops at 0 bar cannot show either (principle P2).

use jk_kernel::{Head, Time};

/// How low (and how high) the pressure went over a window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LowPressure {
    /// The lowest pressure head [m].
    pub lowest: Head,
    /// The highest pressure head [m] - for comparing with the pipes'
    /// rating.
    pub highest: Head,
    /// Time spent below the negative threshold (meaningfully under
    /// atmospheric - not sensor noise around 0).
    pub below_zero: Time,
    /// Time spent at the vapour floor (within the band above it).
    pub at_floor: Time,
}

/// Measures `window` (one sample every `dt` s) against a vapour floor
/// `floor` (gauge) with tolerance `band`; readings below `negative` count
/// as negative pressure.
#[must_use]
pub fn low_pressure(window: &[f64], dt: f64, floor: f64, band: f64, negative: f64) -> LowPressure {
    let lowest = window.iter().copied().fold(f64::INFINITY, f64::min);
    let highest = window.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let count = |pred: &dyn Fn(f64) -> bool| {
        #[allow(clippy::cast_precision_loss)] // window lengths are small
        let n = window.iter().filter(|&&x| pred(x)).count() as f64;
        Time::new(n * dt)
    };
    LowPressure {
        lowest: Head::new(if lowest.is_finite() { lowest } else { 0.0 }),
        highest: Head::new(if highest.is_finite() { highest } else { 0.0 }),
        below_zero: count(&|x| x < negative),
        at_floor: count(&|x| x <= floor + band),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noise_around_zero_is_not_negative_pressure() {
        let x = [0.03, -0.04, 0.01, -0.02, 0.0];
        let low = low_pressure(&x, 0.01, -10.1, 1.0, -1.0);
        assert!(low.below_zero.value().abs() < 1e-12);
        assert!(low.at_floor.value().abs() < 1e-12);
    }

    #[test]
    fn a_spell_at_the_floor_is_also_below_zero() {
        // 1 s at 20 m, 0.5 s at -3 m, 1 s at the floor (-10 m), 0.5 s back up.
        let dt = 0.01;
        let mut x = vec![20.0; 100];
        x.extend(vec![-3.0; 50]);
        x.extend(vec![-10.0; 100]);
        x.extend(vec![5.0; 50]);
        let low = low_pressure(&x, dt, -10.1, 1.0, -1.0);
        assert!((low.lowest.value() + 10.0).abs() < 1e-12);
        assert!((low.highest.value() - 20.0).abs() < 1e-12);
        assert!((low.below_zero.value() - 1.5).abs() < 1e-9);
        assert!((low.at_floor.value() - 1.0).abs() < 1e-9);
    }
}
