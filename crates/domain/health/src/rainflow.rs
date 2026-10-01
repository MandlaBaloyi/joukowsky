//! Rainflow counting: a pressure history into load cycles (ASTM E1049-85,
//! section 5.4.4, the three-point method;
//! `docs/02_physics_of_transients/pressure_leakage_and_fatigue.txt`
//! section 2).
//!
//! A pipe wall does not care when a pressure swing happened, only how big
//! it was and how often. Rainflow pairs every rise with the fall that
//! closes it, so a history becomes a list of cycles (range, mean, count:
//! 1 for a closed cycle, 0.5 for a half). Large swings are kept whole even
//! when small ripples interrupt them - the property fatigue depends on.
//!
//! ```text
//!   keep turning points (peaks and valleys) only;
//!   push each on a stack; while >= 3 on it:
//!     X = |last - second last|, Y = |second last - third last|
//!     X <  Y: stop
//!     X >= Y and the stack holds 3: Y is a HALF cycle; drop the bottom point
//!     X >= Y otherwise:             Y is a FULL cycle; drop the two points of Y
//!   at the end, every range left on the stack is a half cycle.
//! ```
//!
//! The GATE (hysteresis) drops reversals smaller than it before counting:
//! a logger's 2 cm of noise would otherwise add millions of tiny cycles.
//! [`Counter`] does the same live, sample by sample, with the same answer
//! as [`rainflow`] over the whole record.

use crate::HealthError;

/// One counted cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cycle {
    /// Peak-to-valley range [m of pressure head].
    pub range: f64,
    /// Middle of the range [m].
    pub mean: f64,
    /// 1.0 for a full (closed) cycle, 0.5 for a half.
    pub count: f64,
}

/// Live rainflow counting of one sensor's record, fed in pieces.
#[derive(Debug, Clone, PartialEq)]
pub struct Counter {
    gate: f64,
    /// The last confirmed turning point (or the first sample).
    anchor: Option<f64>,
    /// The furthest point since it, and which way (+1 up, -1 down, 0 not
    /// yet clear of the gate).
    extreme: f64,
    direction: i8,
    /// Turning points not yet paired.
    stack: Vec<f64>,
    cycles: Vec<Cycle>,
}

impl Counter {
    /// A counter ignoring reversals smaller than `gate` [m].
    ///
    /// # Errors
    /// A gate that is negative or not finite.
    pub fn new(gate: f64) -> Result<Self, HealthError> {
        if !(gate >= 0.0 && gate.is_finite()) {
            return Err(HealthError::InvalidInput(format!(
                "rainflow gate {gate} must be finite and >= 0"
            )));
        }
        Ok(Self {
            gate,
            anchor: None,
            extreme: 0.0,
            direction: 0,
            stack: Vec::new(),
            cycles: Vec::new(),
        })
    }

    /// Takes the next samples.
    ///
    /// # Errors
    /// A sample that is not finite (nothing after it is taken).
    pub fn push(&mut self, samples: &[f64]) -> Result<(), HealthError> {
        for &x in samples {
            if !x.is_finite() {
                return Err(HealthError::InvalidInput(
                    "a sample is not finite: gaps must be reported, not filled".into(),
                ));
            }
            self.sample(x);
        }
        Ok(())
    }

    fn sample(&mut self, x: f64) {
        let Some(anchor) = self.anchor else {
            self.anchor = Some(x);
            self.extreme = x;
            return;
        };
        match self.direction {
            0 => {
                if (x - anchor).abs() > self.gate {
                    self.turning_point(anchor);
                    self.direction = if x > anchor { 1 } else { -1 };
                    self.extreme = x;
                }
            }
            d => {
                let further = if d > 0 {
                    x > self.extreme
                } else {
                    x < self.extreme
                };
                if further {
                    self.extreme = x;
                } else if (self.extreme - x).abs() > self.gate {
                    let turn = self.extreme;
                    self.turning_point(turn);
                    self.anchor = Some(turn);
                    self.direction = -d;
                    self.extreme = x;
                }
            }
        }
    }

    /// A confirmed turning point: stack it and pair what closes.
    fn turning_point(&mut self, point: f64) {
        self.stack.push(point);
        while self.stack.len() >= 3 {
            let n = self.stack.len();
            let (a, b, c) = (self.stack[n - 3], self.stack[n - 2], self.stack[n - 1]);
            let (x, y) = ((c - b).abs(), (b - a).abs());
            if x < y {
                break;
            }
            let cycle = |count| Cycle {
                range: y,
                mean: f64::midpoint(a, b),
                count,
            };
            if n == 3 {
                self.cycles.push(cycle(0.5));
                self.stack.remove(0);
            } else {
                self.cycles.push(cycle(1.0));
                self.stack.drain(n - 3..n - 1);
            }
        }
    }

    /// Hands over the cycles closed since last asked (the stack's residue
    /// is counted only when the record ends).
    pub fn take_closed(&mut self) -> Vec<Cycle> {
        std::mem::take(&mut self.cycles)
    }

    /// What [`Counter::finish`] would add if the record ended now - the
    /// swings still open, as half cycles - without ending it. A slowly
    /// fading oscillation (each swing smaller than the last) closes no
    /// cycle at all: all its damage sits here until the record ends.
    #[must_use]
    pub fn open(&self) -> Vec<Cycle> {
        let mut rest = self.clone();
        rest.cycles.clear();
        rest.finish()
    }

    /// Ends the record: the last extreme is a turning point, and every
    /// range left unpaired counts as a half cycle. Returns all cycles.
    #[must_use]
    pub fn finish(mut self) -> Vec<Cycle> {
        if self.direction != 0 {
            let last = self.extreme;
            self.turning_point(last);
        }
        let residue: Vec<Cycle> = self
            .stack
            .windows(2)
            .map(|w| Cycle {
                range: (w[1] - w[0]).abs(),
                mean: f64::midpoint(w[0], w[1]),
                count: 0.5,
            })
            .collect();
        self.cycles.extend(residue);
        self.cycles
    }
}

/// Rainflow cycles of a whole record, ignoring reversals smaller than
/// `gate`.
///
/// # Errors
/// A bad gate or a non-finite sample.
pub fn rainflow(samples: &[f64], gate: f64) -> Result<Vec<Cycle>, HealthError> {
    let mut counter = Counter::new(gate)?;
    counter.push(samples)?;
    Ok(counter.finish())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn total(cycles: &[Cycle], range: f64) -> f64 {
        cycles
            .iter()
            .filter(|c| (c.range - range).abs() < 1e-12)
            .map(|c| c.count)
            .sum()
    }

    #[test]
    fn a_ripple_inside_a_big_swing_leaves_the_swing_whole() {
        // 0 -> 10 -> 8 -> 10.5 -> 0: the 2 m ripple is a full cycle; the
        // big swing stays 10.5 m (two halves), not cut into pieces.
        let c = rainflow(&[0.0, 10.0, 8.0, 10.5, 0.0], 0.0).unwrap();
        assert!((total(&c, 2.0) - 1.0).abs() < 1e-12);
        assert!((total(&c, 10.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_fading_ring_closes_nothing_and_stays_open() {
        let x = [10.0, 100.0, 15.0, 95.0, 20.0, 90.0, 25.0];
        let mut c = Counter::new(0.5).unwrap();
        c.push(&x).unwrap();
        assert!(
            c.take_closed().is_empty(),
            "each swing smaller than the last"
        );
        let open = c.open();
        assert_eq!(open.len(), 6);
        assert_eq!(open, c.finish(), "open is what the end would count");
    }

    #[test]
    fn the_gate_drops_the_noise() {
        let noisy: Vec<f64> = (0..1000)
            .map(|i| if i % 2 == 0 { 50.0 } else { 50.03 })
            .collect();
        assert!(rainflow(&noisy, 0.1).unwrap().is_empty());
    }
}
