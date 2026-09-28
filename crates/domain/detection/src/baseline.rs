//! The slowly moving "normal" level a transient is measured against
//! (`event_detection.txt`, Layer 1).
//!
//! Pressure drifts all day as demand changes - metres over minutes. A plain
//! moving average lags behind any trend (by slope x time constant), and that
//! lag looks exactly like a small event. Brown's double exponential
//! smoothing follows the level AND the slope, so a steady trend leaves no
//! lag at all:
//!
//! ```text
//!   s1 = s1 + a (x - s1)          s2 = s2 + a (s1 - s2)
//!   level = 2 s1 - s2             slope = a / (1 - a) (s1 - s2)  per sample
//!   forecast of the next sample = level + slope
//! ```
//!
//! with a = dt / tau. A transient (milliseconds) is far faster than tau
//! (seconds), so it stands out against the forecast while drift does not.

/// Brown's double exponential smoother.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Baseline {
    alpha: f64,
    s1: f64,
    s2: f64,
}

impl Baseline {
    /// Starts at `level` with no trend; `alpha` = dt / tau, in (0, 1].
    #[must_use]
    pub fn new(level: f64, alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(f64::MIN_POSITIVE, 1.0),
            s1: level,
            s2: level,
        }
    }

    /// Expected value of the next sample.
    #[must_use]
    #[inline]
    pub fn forecast(&self) -> f64 {
        let level = 2.0 * self.s1 - self.s2;
        let slope = if self.alpha < 1.0 {
            self.alpha / (1.0 - self.alpha) * (self.s1 - self.s2)
        } else {
            0.0
        };
        level + slope
    }

    /// Takes in a sample judged to be normal.
    #[inline]
    pub fn update(&mut self, x: f64) {
        self.s1 += self.alpha * (x - self.s1);
        self.s2 += self.alpha * (self.s1 - self.s2);
    }

    /// Moves the level by `shift` but keeps the learned trend - after an
    /// event the pressure has a new level, but demand keeps drifting.
    pub fn shift(&mut self, shift: f64) {
        self.s1 += shift;
        self.s2 += shift;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn follows_a_steady_trend_without_lag() {
        let mut b = Baseline::new(60.0, 0.01);
        let mut worst_late = 0.0_f64;
        for k in 0..5000 {
            let x = 60.0 + 0.001 * f64::from(k);
            if k > 3000 {
                worst_late = worst_late.max((b.forecast() - x).abs());
            }
            b.update(x);
        }
        assert!(worst_late < 1e-6, "lag {worst_late}");
    }

    #[test]
    fn shifting_keeps_the_trend() {
        let mut b = Baseline::new(60.0, 0.01);
        for k in 0..5000 {
            b.update(60.0 + 0.001 * f64::from(k));
        }
        let before = b.forecast();
        b.shift(-4.5);
        assert!((b.forecast() - (before - 4.5)).abs() < 1e-9);
    }
}
