//! A synthetic logger: what a real transient logger would report of a
//! simulated pressure history.
//!
//! The same model as the validation tests (`tests/common/sensor.rs`): about
//! 128 samples a second, each the AVERAGE of the pressure over its interval
//! (sensors and anti-alias filters integrate), an arbitrary clock phase,
//! and Gaussian noise. Seeded, so every run is identical.

use jk_kernel::random::Rng;

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

/// Samples `truth` (one value every `dt` seconds from t = 0, joined by
/// straight lines) the way `logger` would. Returns nothing for a history
/// shorter than two values.
pub fn sample(truth: &[f64], dt: f64, logger: &Logger, rng: &mut Rng) -> Vec<f64> {
    if truth.len() < 2 {
        return Vec::new();
    }
    // Running integral of the piecewise-linear truth at its own samples.
    let mut cumulative = Vec::with_capacity(truth.len());
    cumulative.push(0.0);
    for w in truth.windows(2) {
        let last = cumulative[cumulative.len() - 1];
        cumulative.push(last + f64::midpoint(w[0], w[1]) * dt);
    }
    let end = to_f64(truth.len() - 1) * dt;
    let integral = |t: f64| -> f64 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped
        let i = ((t / dt).floor().max(0.0) as usize).min(truth.len() - 2);
        let f = t / dt - to_f64(i);
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

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}
