//! Two-sided CUSUM change detection (Page 1954).
//!
//! `docs/04_detection_and_localisation/event_detection.txt`, Layer 2(a).
//! Each new sample arrives as a standardised residual z (distance from the
//! baseline, in units of the noise). Two running sums collect evidence of
//! a rise and of a drop:
//!
//! ```text
//!   S+ = max(0, S+ + z - k)        S- = max(0, S- - z - k)
//!   alarm when S+ > h or S- > h
//! ```
//!
//! k (the drift) ignores shifts smaller than k noise units; h (the
//! threshold) sets the trade-off between detection delay and false alarms.
//! For a step of d noise units the delay is about h / (d - k) samples.
//! The sum drops back to zero whenever the evidence runs out, so the last
//! sample at which it was zero estimates when the change began.
//!
//! Nothing is allocated per sample: this runs on a gateway as easily as on
//! a server (principle P6).

use crate::error::{DetectionError, positive};

/// CUSUM settings, both in units of the sensor's noise standard deviation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CusumConfig {
    /// k: shifts smaller than this are ignored.
    pub drift: f64,
    /// h: evidence needed before raising an alarm.
    pub threshold: f64,
}

impl Default for CusumConfig {
    /// k = 3, h = 10: for Gaussian noise a false alarm is practically never
    /// raised, while a 0.2-bar step on a sensor with 2 cm of noise (100
    /// noise units) is caught on its first sample.
    fn default() -> Self {
        Self {
            drift: 3.0,
            threshold: 10.0,
        }
    }
}

/// Which way the signal moved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Pressure went up (e.g. a valve closing upstream of the sensor).
    Rise,
    /// Pressure went down (e.g. a burst, a pump trip).
    Drop,
}

/// An alarm: where it was raised and where the change is estimated to begin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Alarm {
    /// Index of the sample that raised the alarm.
    pub index: usize,
    /// Index of the first sample of the change (the sample after the sum
    /// was last zero).
    pub change_start: usize,
    /// Direction of the change.
    pub direction: Direction,
}

/// A running two-sided CUSUM.
#[derive(Debug, Clone, PartialEq)]
pub struct Cusum {
    config: CusumConfig,
    rise: f64,
    drop: f64,
    rise_zero_at: usize,
    drop_zero_at: usize,
    next: usize,
}

impl Cusum {
    /// A detector starting at sample index `first`.
    ///
    /// # Errors
    /// If drift or threshold are not finite and positive.
    pub fn new(config: CusumConfig, first: usize) -> Result<Self, DetectionError> {
        positive("cusum drift", config.drift)?;
        positive("cusum threshold", config.threshold)?;
        Ok(Self {
            config,
            rise: 0.0,
            drop: 0.0,
            rise_zero_at: first.saturating_sub(1),
            drop_zero_at: first.saturating_sub(1),
            next: first,
        })
    }

    /// Feeds one standardised residual; returns an alarm if one is raised.
    /// After an alarm the sums are cleared, ready to re-arm.
    #[inline]
    pub fn push(&mut self, z: f64) -> Option<Alarm> {
        let index = self.next;
        self.next += 1;
        self.rise = (self.rise + z - self.config.drift).max(0.0);
        self.drop = (self.drop - z - self.config.drift).max(0.0);
        if self.rise == 0.0 {
            self.rise_zero_at = index;
        }
        if self.drop == 0.0 {
            self.drop_zero_at = index;
        }
        let alarm = if self.rise > self.config.threshold {
            Some((Direction::Rise, self.rise_zero_at))
        } else if self.drop > self.config.threshold {
            Some((Direction::Drop, self.drop_zero_at))
        } else {
            None
        };
        alarm.map(|(direction, zero_at)| {
            self.reset_at(index + 1);
            Alarm {
                index,
                change_start: zero_at + 1,
                direction,
            }
        })
    }

    /// The earliest sample at which a change that a FUTURE alarm reports
    /// could begin: while evidence is building up, where it started;
    /// otherwise the next sample. (Lets a stream say which of its past is
    /// already final.)
    #[must_use]
    pub fn pending_start(&self) -> usize {
        let rise = if self.rise > 0.0 {
            self.rise_zero_at + 1
        } else {
            self.next
        };
        let drop = if self.drop > 0.0 {
            self.drop_zero_at + 1
        } else {
            self.next
        };
        rise.min(drop)
    }

    /// Clears the evidence; the next sample pushed has index `next`.
    pub fn reset_at(&mut self, next: usize) {
        self.rise = 0.0;
        self.drop = 0.0;
        self.rise_zero_at = next.saturating_sub(1);
        self.drop_zero_at = next.saturating_sub(1);
        self.next = next;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_step_is_caught_and_its_start_found() {
        let mut c = Cusum::new(CusumConfig::default(), 0).unwrap();
        for _ in 0..50 {
            assert_eq!(c.push(0.0), None);
        }
        // Step of 6 noise units at sample 50: evidence grows 3 per sample,
        // so the sum passes 10 on the fourth step sample (index 53).
        let mut alarm = None;
        for _ in 0..10 {
            if let Some(a) = c.push(-6.0) {
                alarm = Some(a);
                break;
            }
        }
        let a = alarm.expect("step must be detected");
        assert_eq!(a.direction, Direction::Drop);
        assert_eq!(a.change_start, 50);
        assert_eq!(a.index, 53);
    }

    #[test]
    fn shifts_below_the_drift_never_alarm() {
        let mut c = Cusum::new(CusumConfig::default(), 0).unwrap();
        for _ in 0..10_000 {
            assert_eq!(c.push(2.9), None);
        }
    }

    #[test]
    fn rejects_bad_settings() {
        let bad = CusumConfig {
            drift: 0.0,
            threshold: 10.0,
        };
        assert!(Cusum::new(bad, 0).is_err());
    }
}
