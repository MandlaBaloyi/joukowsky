//! The initial front: how far it swung and how sharp it was.

/// Largest excursion from `level` in direction `sign` (+1 rise, -1 drop)
/// over `window`, signed.
#[must_use]
pub fn swing(window: &[f64], level: f64, sign: f64) -> f64 {
    let extreme = window
        .iter()
        .map(|x| sign * (x - level))
        .fold(0.0_f64, f64::max);
    sign * extreme
}

/// 10-90% rise time of a swing, in samples (fractional, interpolated):
/// the time from first reaching 10% of `swing` to first reaching 90%.
/// A sharp step sampled by an integrating logger shows one in-between
/// sample, so its front spans two intervals: 1.6 samples (0.8 x 2); a pump
/// running down gives tens.
#[must_use]
pub fn rise_samples(window: &[f64], level: f64, swing: f64) -> f64 {
    if swing == 0.0 {
        return 0.0;
    }
    let progress = |x: f64| (x - level) / swing; // 0 at rest, 1 at the extreme
    let crossing = |target: f64| -> f64 {
        for i in 1..window.len() {
            let (a, b) = (progress(window[i - 1]), progress(window[i]));
            if a < target && b >= target {
                #[allow(clippy::cast_precision_loss)] // window indices are small
                return (i - 1) as f64 + (target - a) / (b - a);
            }
        }
        #[allow(clippy::cast_precision_loss)]
        let end = window.len() as f64;
        end
    };
    (crossing(0.9) - crossing(0.1)).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_linear_ramp_rises_over_eighty_percent_of_its_length() {
        // 60 -> 50 over 100 samples, then flat.
        let x: Vec<f64> = (0..200)
            .map(|i: i32| 60.0 - 10.0 * (f64::from(i) / 100.0).min(1.0))
            .collect();
        let s = swing(&x, 60.0, -1.0);
        assert!((s + 10.0).abs() < 1e-12);
        assert!((rise_samples(&x, 60.0, s) - 80.0).abs() < 1e-9);
    }

    #[test]
    fn an_integrated_step_rises_in_one_point_six_samples() {
        // One in-between sample (55): 10% is reached at 1.2, 90% at 2.8.
        let x = [60.0, 60.0, 55.0, 50.0, 50.0];
        let r = rise_samples(&x, 60.0, swing(&x, 60.0, -1.0));
        assert!((r - 1.6).abs() < 1e-12, "{r}");
    }
}
