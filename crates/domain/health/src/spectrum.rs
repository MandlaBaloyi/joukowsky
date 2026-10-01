//! A running histogram of counted cycles by range - what a sensor's pipes
//! have taken over months, in bounded memory.
//!
//! Ranges are binned (bin `w`: [k w, (k+1) w)) and each bin stands for its
//! centre: the equivalent cycles of a swing of range r are then off by at
//! most about m w / (2 r) - 2% for 10 m swings in 0.1 m bins with m = 4.

use crate::HealthError;
use crate::rainflow::Cycle;

/// Cycles counted per range bin.
#[derive(Debug, Clone, PartialEq)]
pub struct Spectrum {
    bin: f64,
    counts: Vec<f64>,
}

impl Spectrum {
    /// An empty spectrum with bins `bin` [m] wide.
    ///
    /// # Errors
    /// A bin that is not finite and positive.
    pub fn new(bin: f64) -> Result<Self, HealthError> {
        if !(bin > 0.0 && bin.is_finite()) {
            return Err(HealthError::InvalidInput(format!(
                "spectrum bin {bin} must be finite and positive"
            )));
        }
        Ok(Self {
            bin,
            counts: Vec::new(),
        })
    }

    /// Adds counted cycles.
    pub fn add(&mut self, cycles: &[Cycle]) {
        for c in cycles {
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // range >= 0
            let k = (c.range / self.bin).floor() as usize;
            if k >= self.counts.len() {
                self.counts.resize(k + 1, 0.0);
            }
            self.counts[k] += c.count;
        }
    }

    /// (bin centre [m], cycles) for every non-empty bin, smallest first.
    pub fn bins(&self) -> impl Iterator<Item = (f64, f64)> + '_ {
        #[allow(clippy::cast_precision_loss)] // bin counts are small
        self.counts
            .iter()
            .enumerate()
            .filter(|(_, n)| **n > 0.0)
            .map(|(k, n)| ((k as f64 + 0.5) * self.bin, *n))
    }

    /// All cycles counted.
    #[must_use]
    pub fn total(&self) -> f64 {
        self.counts.iter().fold(0.0, |a, n| a + n)
    }

    /// The cycles of `reference_range` doing the same damage, for S-N
    /// exponent `exponent` (see [`crate::damage::equivalent_cycles`]).
    #[must_use]
    pub fn equivalent_cycles(&self, reference_range: f64, exponent: f64) -> f64 {
        self.bins()
            .map(|(r, n)| n * (r / reference_range).powf(exponent))
            .fold(0.0, |a, n| a + n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bins_stand_for_their_centres() {
        let mut s = Spectrum::new(1.0).unwrap();
        s.add(&[
            Cycle {
                range: 9.2,
                mean: 0.0,
                count: 1.0,
            },
            Cycle {
                range: 9.9,
                mean: 0.0,
                count: 0.5,
            },
            Cycle {
                range: 0.3,
                mean: 0.0,
                count: 0.5,
            },
        ]);
        let bins: Vec<(f64, f64)> = s.bins().collect();
        assert_eq!(bins, vec![(0.5, 0.5), (9.5, 1.5)]);
        assert!((s.total() - 2.0).abs() < 1e-12);
        assert!((s.equivalent_cycles(9.5, 3.0) - (1.5 + 0.5 / 19.0_f64.powi(3))).abs() < 1e-12);
    }
}
