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

#[cfg(test)]
mod tests {
    use super::*;

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
