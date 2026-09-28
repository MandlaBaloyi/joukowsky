//! When exactly did the front arrive? (Layer 3: arrival-time refinement.)
//!
//! Localisation turns timing errors straight into distance errors
//! (dx = a dt / 2): at 128 samples/s one sample is 7.8 ms, i.e. ~4 m at
//! a = 1000 m/s. So the arrival time must be found to a FRACTION of a
//! sample.
//!
//! Method: the step AREA. Real loggers report the average pressure over
//! each sampling interval (the sensor and its anti-alias filter integrate).
//! A sharp front arriving part-way through an interval therefore produces
//! one in-between sample, and the shortfall of that sample below the new
//! level says exactly how far into the interval the front came. Summing the
//! shortfalls over a window around the front,
//!
//! ```text
//!   t_front = t_first - T + T * sum_j (1 - (x_j - pre) / (post - pre))
//! ```
//!
//! (samples wholly before the front add exactly 1, wholly after add 0, so
//! the window can be generous). For an integrating sensor this is EXACT for
//! a sharp step; for a front spread over several samples it gives the
//! front's centre of change - the same definition at every sensor, which is
//! what matters when arrival times are compared.
//!
//! Joining the dots (interpolating a half-level crossing) is biased by up to
//! ~9% of a sample depending on the sampling phase; the area method is not.

use crate::noise::median;

/// Windows used around an alarm, in samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OnsetConfig {
    /// Samples before the change used to measure the old level.
    pub before: usize,
    /// Samples after the alarm used to measure the new level.
    pub after: usize,
    /// Extra samples before the estimated change start included in the
    /// area sum (in case the change began a little earlier).
    pub margin: usize,
}

impl Default for OnsetConfig {
    fn default() -> Self {
        Self {
            before: 16,
            after: 6,
            margin: 3,
        }
    }
}

/// A picked arrival.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Onset {
    /// Arrival time as a fractional sample index (the time axis of the
    /// samples: sample k is the average over the interval ending at k).
    pub index: f64,
    /// One-standard-deviation uncertainty of `index`, in samples.
    pub uncertainty: f64,
    /// Level before the front.
    pub pre: f64,
    /// Level after the front.
    pub post: f64,
}

/// Picks the arrival around an alarm raised at `alarm` whose change was
/// estimated to start at `change_start`, given the sensor noise `sigma`.
/// Returns None if the windows do not fit in `samples` or the step is lost
/// in the noise.
#[must_use]
pub fn pick(
    samples: &[f64],
    change_start: usize,
    alarm: usize,
    config: &OnsetConfig,
    sigma: f64,
) -> Option<Onset> {
    let first = change_start.checked_sub(config.margin)?;
    let pre_start = first.checked_sub(config.before)?;
    let post_end = alarm + 1 + config.after;
    if post_end > samples.len() || config.before == 0 || config.after == 0 {
        return None;
    }
    let pre = median(&mut samples[pre_start..first].to_vec());
    let post = median(&mut samples[alarm + 1..post_end].to_vec());
    let step = post - pre;
    if step.abs() <= 3.0 * sigma {
        return None;
    }
    let window = &samples[first..=alarm];
    let shortfall: f64 = window.iter().map(|x| 1.0 - (x - pre) / step).sum();
    #[allow(clippy::cast_precision_loss)] // sample indices are far below 2^52
    let index = first as f64 - 1.0 + shortfall;
    // Each term carries noise sigma/|step|; the two levels add their own.
    #[allow(clippy::cast_precision_loss)]
    let n = window.len() as f64;
    #[allow(clippy::cast_precision_loss)]
    let level_noise = 1.253 * sigma / (config.before.min(config.after) as f64).sqrt();
    let uncertainty = (sigma * n.sqrt() + level_noise * n) / step.abs();
    Some(Onset {
        index,
        uncertainty,
        pre,
        post,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An integrating sensor sampling a step from `pre` to `post` at time
    /// `t_step` (in samples): sample k averages the interval (k-1, k].
    fn integrated_step(n: usize, t_step: f64, pre: f64, post: f64) -> Vec<f64> {
        (0..n)
            .map(|k| {
                #[allow(clippy::cast_precision_loss)]
                let k = k as f64;
                let covered = (k - t_step).clamp(0.0, 1.0);
                pre + (post - pre) * covered
            })
            .collect()
    }

    #[test]
    fn exact_for_a_sharp_step_at_any_phase() {
        for i in 0..20 {
            let t_step = 40.0 + f64::from(i) / 20.0;
            let x = integrated_step(80, t_step, 60.0, 55.5);
            let o = pick(&x, 40, 42, &OnsetConfig::default(), 0.01).unwrap();
            assert!(
                (o.index - t_step).abs() < 1e-12,
                "phase {i}: {} vs {t_step}",
                o.index
            );
        }
    }

    #[test]
    fn a_ramp_front_is_placed_at_its_centre() {
        // Linear ramp from sample 40 to 44: centre of change at 42 (in the
        // integrated-sample time axis, 42.5 - 0.5 for the averaging).
        let x: Vec<f64> = (0..80)
            .map(|k: i32| {
                let k = f64::from(k);
                60.0 - 4.0 * ((k - 40.0) / 4.0).clamp(0.0, 1.0)
            })
            .collect();
        let o = pick(&x, 41, 44, &OnsetConfig::default(), 0.01).unwrap();
        assert!((o.index - 41.5).abs() < 1e-12, "got {}", o.index);
    }

    #[test]
    fn declines_when_the_windows_do_not_fit() {
        let x = integrated_step(30, 5.0, 60.0, 55.0);
        assert!(pick(&x, 5, 6, &OnsetConfig::default(), 0.01).is_none());
    }
}
