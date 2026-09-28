//! Synthetic sensors: turning a simulated pressure history into what a real
//! logger would report.
//!
//! A commercial transient logger (docs `03_sensing_and_signal`) samples at
//! about 128 S/s, integrates over each interval (the sensor and its
//! anti-alias filter average), starts at an arbitrary clock phase, and adds
//! measurement noise. Testing the detector on such data - where the true
//! event time and place are known exactly - is validation rung 3.
//!
//! Randomness comes from the kernel's own generator
//! ([`jk_kernel::random::Rng`]) with fixed seeds, so every run of every test
//! is identical.

pub use jk_kernel::random::Rng;

/// How a synthetic logger samples.
#[derive(Debug, Clone, Copy)]
pub struct Logger {
    /// Samples per second.
    pub rate: f64,
    /// Time of the first sample [s] (the clock phase).
    pub phase: f64,
    /// Standard deviation of the measurement noise [m].
    pub noise: f64,
}

/// Samples a simulated head history `truth` (one value every `dt` seconds
/// from t = 0, joined by straight lines) the way `logger` would: sample k,
/// at time `phase + k / rate`, is the AVERAGE of the truth over the
/// interval ending there, plus noise. Samples stop where the truth ends.
///
/// # Panics
/// If `truth` has fewer than 2 values.
#[must_use]
pub fn sample(truth: &[f64], dt: f64, logger: &Logger, rng: &mut Rng) -> Vec<f64> {
    assert!(truth.len() >= 2, "need a history to sample");
    // Running integral of the piecewise-linear truth at its own samples.
    let mut cumulative = Vec::with_capacity(truth.len());
    cumulative.push(0.0);
    for w in truth.windows(2) {
        let last = *cumulative.last().expect("non-empty");
        cumulative.push(last + f64::midpoint(w[0], w[1]) * dt);
    }
    #[allow(clippy::cast_precision_loss)]
    let end = (truth.len() - 1) as f64 * dt;
    let integral = |t: f64| -> f64 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped
        let i = ((t / dt).floor().max(0.0) as usize).min(truth.len() - 2);
        #[allow(clippy::cast_precision_loss)]
        let f = t / dt - i as f64;
        let (a, b) = (truth[i], truth[i + 1]);
        cumulative[i] + dt * (a * f + 0.5 * (b - a) * f * f)
    };
    let interval = 1.0 / logger.rate;
    let mut out = Vec::new();
    let mut k = 0_u32;
    loop {
        let t = logger.phase + f64::from(k) * interval;
        if t > end {
            break;
        }
        let from = (t - interval).max(0.0);
        let average = if t > from {
            (integral(t) - integral(from)) / (t - from)
        } else {
            truth[0]
        };
        out.push(average + logger.noise * rng.normal());
        k += 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn averaging_a_step_gives_the_covered_fraction() {
        // Truth every 0.5 ms: 60 up to sample 2005 (t = 1.0025 s), 50 from
        // sample 2006 (t = 1.003 s), a straight line between. A 100 Hz
        // sample at t = 1.01 averages (1.00, 1.01]: 2.5 ms at 60, the
        // 0.5 ms ramp (mean 55), then 7 ms at 50.
        let dt = 0.0005;
        let truth: Vec<f64> = (0..4000)
            .map(|i| if i <= 2005 { 60.0 } else { 50.0 })
            .collect();
        let logger = Logger {
            rate: 100.0,
            phase: 0.0,
            noise: 0.0,
        };
        let x = sample(&truth, dt, &logger, &mut Rng::new(1));
        let expected = (60.0 * 0.0025 + 55.0 * 0.0005 + 50.0 * 0.007) / 0.01;
        assert!((x[101] - expected).abs() < 1e-9, "{} vs {expected}", x[101]);
    }
}
