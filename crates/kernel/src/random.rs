//! Deterministic random numbers.
//!
//! Stochastic methods (Monte Carlo over uncertain wave speeds, synthetic
//! test data) must be replayable bit for bit (rule T3, principle P5): the
//! same seed always gives the same numbers, on every run and machine. So
//! the generator is our own, small and fully specified: `SplitMix64`
//! (Steele, Lea & Flood 2014) for bits, Box-Muller for normal numbers.

/// A seeded `SplitMix64` generator.
#[derive(Debug, Clone, PartialEq)]
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    /// A generator from `seed`.
    #[must_use]
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed,
            spare: None,
        }
    }

    /// Next 64 random bits.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    #[allow(clippy::cast_precision_loss)] // 53 random bits, exact
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal (mean 0, standard deviation 1), by Box-Muller.
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let u1 = 1.0 - self.uniform(); // in (0, 1]: the log is finite
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = std::f64::consts::TAU * u2;
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_numbers() {
        let (mut a, mut b) = (Rng::new(42), Rng::new(42));
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn normal_numbers_have_the_right_spread() {
        let mut rng = Rng::new(7);
        let n = 20_000;
        let xs: Vec<f64> = (0..n).map(|_| rng.normal()).collect();
        let mean = xs.iter().sum::<f64>() / f64::from(n);
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / f64::from(n);
        assert!(
            mean.abs() < 0.03 && (var - 1.0).abs() < 0.05,
            "mean {mean}, var {var}"
        );
    }
}
