//! Scoring one candidate against the observed arrival times.

/// Result of scoring one candidate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Score {
    /// log-likelihood, -chi^2 / 2 (without constant terms).
    pub log_likelihood: f64,
    /// Best emission time t0 for this candidate (s).
    pub emission: f64,
    /// chi^2: sum of squared standardised residuals.
    pub chi_squared: f64,
}

/// Scores a candidate from observed arrival times `observed`, their
/// standard deviations `sigma`, and predicted travel times `travel` (one
/// per sensor). An unreachable candidate (infinite travel time) scores
/// minus infinity.
#[must_use]
pub fn score(observed: &[f64], sigma: &[f64], travel: &[f64]) -> Score {
    let (mut sw, mut swr) = (0.0, 0.0);
    for ((t, s), tt) in observed.iter().zip(sigma).zip(travel) {
        let w = 1.0 / (s * s);
        sw += w;
        swr += w * (t - tt);
    }
    let emission = swr / sw;
    let chi_squared: f64 = observed
        .iter()
        .zip(sigma)
        .zip(travel)
        .map(|((t, s), tt)| {
            let r = (t - emission - tt) / s;
            r * r
        })
        .sum();
    if !chi_squared.is_finite() {
        return Score {
            log_likelihood: f64::NEG_INFINITY,
            emission: f64::NAN,
            chi_squared: f64::INFINITY,
        };
    }
    Score {
        log_likelihood: -0.5 * chi_squared,
        emission,
        chi_squared,
    }
}

/// Scores a candidate when the arrival-time errors are CORRELATED, with
/// covariance `cov` (n x n, row-major; destroyed - it is factored in
/// place). This is how uncertain wave speeds enter: two sensors whose waves
/// travelled down the same pipe share that pipe's travel-time error.
///
/// With r = observed - travel and 1 the vector of ones, the emission time
/// t0 is the generalised least-squares estimate, and marginalising t0 and
/// the Gaussian errors gives
///
/// ```text
///   t0   = 1' C^-1 r / 1' C^-1 1
///   chi2 = r' C^-1 r - (1' C^-1 r)^2 / 1' C^-1 1
///   log L = -chi2/2 - (log det C)/2 - (log 1' C^-1 1)/2
/// ```
///
/// The last two terms matter because C changes from candidate to candidate
/// (a longer route through uncertain pipes is a vaguer prediction).
#[must_use]
pub fn score_correlated(observed: &[f64], travel: &[f64], cov: &mut [f64]) -> Score {
    let n = observed.len();
    let unreachable = Score {
        log_likelihood: f64::NEG_INFINITY,
        emission: f64::NAN,
        chi_squared: f64::INFINITY,
    };
    if travel.iter().any(|t| !t.is_finite()) || jk_kernel::linalg::cholesky(cov, n).is_err() {
        return unreachable;
    }
    let r: Vec<f64> = observed.iter().zip(travel).map(|(t, tt)| t - tt).collect();
    let mut c_inv_r = r.clone();
    jk_kernel::linalg::solve_factored(cov, &mut c_inv_r, n);
    let mut c_inv_1 = vec![1.0; n];
    jk_kernel::linalg::solve_factored(cov, &mut c_inv_1, n);
    let one_c_1: f64 = c_inv_1.iter().sum();
    let one_c_r: f64 = c_inv_r.iter().sum();
    let r_c_r: f64 = r.iter().zip(&c_inv_r).map(|(a, b)| a * b).sum();
    let chi_squared = (r_c_r - one_c_r * one_c_r / one_c_1).max(0.0);
    Score {
        log_likelihood: -0.5 * chi_squared
            - 0.5 * jk_kernel::linalg::log_det_factored(cov, n)
            - 0.5 * one_c_1.ln(),
        emission: one_c_r / one_c_1,
        chi_squared,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_perfect_candidate_scores_zero_misfit_and_recovers_t0() {
        // Emitted at t0 = 2.0; travel 0.7 and 1.3 s.
        let s = score(&[2.7, 3.3], &[1e-3, 1e-3], &[0.7, 1.3]);
        assert!(s.chi_squared < 1e-18);
        assert!((s.emission - 2.0).abs() < 1e-12);
    }

    #[test]
    fn with_independent_errors_the_correlated_score_matches_the_simple_one() {
        let (obs, sig, tr) = ([2.71, 3.29, 2.4], [1e-3, 2e-3, 1.5e-3], [0.7, 1.3, 0.41]);
        let simple = score(&obs, &sig, &tr);
        let mut cov = vec![0.0; 9];
        for i in 0..3 {
            cov[i * 3 + i] = sig[i] * sig[i];
        }
        let full = score_correlated(&obs, &tr, &mut cov);
        assert!((full.chi_squared - simple.chi_squared).abs() < 1e-9 * simple.chi_squared);
        assert!((full.emission - simple.emission).abs() < 1e-12);
    }

    #[test]
    fn a_wrong_candidate_is_penalised() {
        let good = score(&[2.7, 3.3], &[1e-3, 1e-3], &[0.7, 1.3]);
        let bad = score(&[2.7, 3.3], &[1e-3, 1e-3], &[0.75, 1.25]);
        assert!(bad.log_likelihood < good.log_likelihood - 1000.0);
    }
}
