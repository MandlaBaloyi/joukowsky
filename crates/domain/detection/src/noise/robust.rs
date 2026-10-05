//! How much a quiet sensor wiggles.
//!
//! Every detection threshold is set in units of the sensor's own noise
//! (docs: "thresholds are set per sensor from its own noise statistics"),
//! so this estimate matters. It must not be fooled by:
//!   - slow drift (demand changing over the day) - hence it works on the
//!     DIFFERENCES between neighbouring samples, which drift barely touches;
//!   - the odd spike or small event - hence the MEDIAN absolute deviation
//!     (MAD), not the standard deviation: a few outliers cannot move it.
//!
//! For white noise of standard deviation sigma, neighbouring differences
//! have standard deviation sigma * sqrt(2), and for a normal distribution
//! the standard deviation is 1.4826 x MAD. Hence
//!
//! ```text
//!   sigma = 1.4826 * MAD(x[k] - x[k-1]) / sqrt(2)
//! ```

/// 1 / Phi^-1(3/4): turns a normal distribution's MAD into its standard
/// deviation.
const MAD_TO_SIGMA: f64 = 1.482_602_218_505_602;

/// Median of `values` (sorted in place). NaN-free input expected.
///
/// # Panics
/// If `values` is empty.
#[must_use]
pub fn median(values: &mut [f64]) -> f64 {
    assert!(!values.is_empty(), "median of nothing");
    values.sort_by(f64::total_cmp);
    let n = values.len();
    if n % 2 == 1 {
        values[n / 2]
    } else {
        f64::midpoint(values[n / 2 - 1], values[n / 2])
    }
}

/// Robust noise standard deviation of `samples`, from first differences.
///
/// # Panics
/// If fewer than 3 samples are given.
#[must_use]
pub fn sigma_from_differences(samples: &[f64]) -> f64 {
    assert!(samples.len() >= 3, "need at least 3 samples");
    let mut diffs: Vec<f64> = samples.windows(2).map(|w| w[1] - w[0]).collect();
    let centre = median(&mut diffs);
    let mut deviations: Vec<f64> = diffs.iter().map(|d| (d - centre).abs()).collect();
    MAD_TO_SIGMA * median(&mut deviations) / std::f64::consts::SQRT_2
}

/// Robust standard deviation of `samples` about their straight-line trend
/// (least squares): 1.4826 x MAD of what the line leaves. Noise that
/// WANDERS - pump pulsation, a rig sloshing, swings of a second or so -
/// barely shows in neighbouring differences but counts in full here
/// (FN-047); a steady ramp (a pipe refilling, demand drifting) is taken
/// out by the line and counts not at all.
///
/// # Panics
/// If fewer than 3 samples are given.
#[must_use]
pub fn sigma_about_trend(samples: &[f64]) -> f64 {
    assert!(samples.len() >= 3, "need at least 3 samples");
    #[allow(clippy::cast_precision_loss)] // window lengths are small
    let n = samples.len() as f64;
    let mid = (n - 1.0) / 2.0;
    let mean = samples.iter().sum::<f64>() / n;
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for (k, v) in (0_u32..).zip(samples) {
        let x = f64::from(k) - mid;
        sxy += x * (v - mean);
        sxx += x * x;
    }
    let slope = sxy / sxx;
    let mut left: Vec<f64> = (0_u32..)
        .zip(samples)
        .map(|(k, v)| v - mean - slope * (f64::from(k) - mid))
        .collect();
    let centre = median(&mut left);
    let mut deviations: Vec<f64> = left.iter().map(|r| (r - centre).abs()).collect();
    MAD_TO_SIGMA * median(&mut deviations)
}

/// How much larger than the jitter the spread about the trend must be to
/// count as wander. For white noise the two estimates agree within their
/// sampling spread (about 7% each over 2 s at 128 S/s), so 1.5 is never
/// reached by chance; the real rig's wander was 3-7 times its jitter
/// (FN-047).
pub const WANDER_RATIO: f64 = 1.5;

/// The sensor's noise unit over a quiet window: the jitter (neighbouring
/// differences) - unless the spread about the window's trend is more than
/// [`WANDER_RATIO`] times it: then the noise WANDERS and that spread is the
/// honest unit (FN-047). Taking simply the larger of the two would raise
/// the unit of white noise by its sampling spread, a few percent, and
/// lose events at the edge of detection (a refill's first rise, V-S6).
///
/// # Panics
/// If fewer than 3 samples are given.
#[must_use]
pub fn noise_unit(samples: &[f64]) -> f64 {
    let jitter = sigma_from_differences(samples);
    let spread = sigma_about_trend(samples);
    if spread > WANDER_RATIO * jitter {
        spread
    } else {
        jitter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wander_hides_from_differences_but_not_from_the_trend() {
        // A slow swing of 1 cm amplitude (period 1 s at 128 S/s) on a
        // steady ramp of 1 m in 2 s, no jitter at all: neighbouring
        // differences see almost nothing; the spread about the trend sees
        // the swing (RMS 0.71 cm, MAD-based about 1 cm) and not the ramp.
        let swing: Vec<f64> = (0..256)
            .map(|k| {
                let t = f64::from(k) / 128.0;
                0.5 * t + 0.01 * (2.0 * std::f64::consts::PI * t).sin()
            })
            .collect();
        assert!(sigma_from_differences(&swing) < 0.001);
        let s = sigma_about_trend(&swing);
        assert!((0.006..0.012).contains(&s), "{s}");
        let ramp: Vec<f64> = (0..256).map(|k| 0.5 * f64::from(k) / 128.0).collect();
        assert!(noise_unit(&ramp) < 1e-9);
    }

    #[test]
    fn median_of_odd_and_even_counts() {
        assert!((median(&mut [3.0, 1.0, 2.0]) - 2.0).abs() < f64::EPSILON);
        assert!((median(&mut [4.0, 1.0, 3.0, 2.0]) - 2.5).abs() < f64::EPSILON);
    }

    #[test]
    fn a_straight_ramp_has_no_noise() {
        // Drift alone (a steady trend) must not look like noise.
        let ramp: Vec<f64> = (0..100).map(|k| 50.0 + 0.01 * f64::from(k)).collect();
        assert!(sigma_from_differences(&ramp) < 1e-12);
    }
}
