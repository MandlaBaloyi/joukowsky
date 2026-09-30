//! A leak alarm: a lasting rise in night flow beyond how much nights vary.
//!
//! The baseline is the first `baseline_nights` present nights: their mean
//! mu and night-to-night spread s. From then on each night gives
//! z = (flow - mu) / s, and a one-sided CUSUM (Page 1954)
//!
//! ```text
//!   S_k = max(0, S_{k-1} + z_k - drift)      alarm when S_k > threshold
//! ```
//!
//! rises only when nights sit above the baseline night after night. The
//! alarm dates the leak to the night after S last stood at zero and sizes
//! it as the mean rise since then, with its standard error (P3). Missing
//! nights are skipped - they neither raise nor lower S (P6).
//!
//! A meter's calibration error shifts every night alike: it cancels here
//! and only blurs the ABSOLUTE size (reported as the size's uncertainty
//! from `meter_bias`).

use jk_kernel::{Flow, Time};

use crate::BalanceError;
use crate::night::Night;

/// How the watch runs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeakOptions {
    /// Present nights that make the baseline. Default 28: four whole weeks
    /// (weekday and weekend nights differ), and a baseline estimated from
    /// fewer nights raises false alarms (FN-025).
    pub baseline_nights: usize,
    /// CUSUM drift, in spreads. Default 0.5.
    pub drift: f64,
    /// CUSUM alarm level, in spreads. Default 6, chosen by simulation
    /// (tests/balance): about one false alarm per 590 nights (1.6 years) a
    /// zone, while a leak of 1.2 spreads is caught within a week 72% of
    /// the time. Page's classic 5 gave one every 267-413 nights.
    pub threshold: f64,
    /// Possible calibration error of the balance as a whole, relative to
    /// the night flow (e.g. 0.02 for meters good to 2%). Default 0.02.
    pub meter_bias: f64,
}

impl Default for LeakOptions {
    fn default() -> Self {
        Self {
            baseline_nights: 28,
            drift: 0.5,
            threshold: 6.0,
            meter_bias: 0.02,
        }
    }
}

/// A leak found.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeakAlarm {
    /// The first night of the rise (as dated by the CUSUM).
    pub first_night: Time,
    /// The night the alarm was raised.
    pub raised: Time,
    /// Estimated leak: mean night flow since the first night minus the
    /// baseline.
    pub size: Flow,
    /// One-sigma uncertainty of the size: the spread over the nights
    /// averaged, the baseline's own uncertainty, and the meters' possible
    /// calibration error.
    pub uncertainty: Flow,
}

/// What the watch found over the nights given.
#[derive(Debug, Clone, PartialEq)]
pub struct Watch {
    /// Baseline night flow mu [m^3/s].
    pub baseline: Flow,
    /// Night-to-night spread s [m^3/s].
    pub spread: Flow,
    /// The smallest leak caught within a week, with this spread
    /// ([`smallest_detectable`]).
    pub smallest_in_a_week: Flow,
    /// Leak alarms, in order (after an alarm the watch restarts on a new
    /// baseline: the leak may be repaired, or a second one may come).
    pub alarms: Vec<LeakAlarm>,
    /// The CUSUM after each night (NaN for missing or baseline nights).
    pub cusum: Vec<f64>,
}

/// The smallest step in night flow caught within `nights` nights, for a
/// night-to-night spread `spread`: the CUSUM reaches `threshold` after n
/// nights when n (d / s - drift) >= threshold, so d = s (drift +
/// threshold / n).
#[must_use]
pub fn smallest_detectable(spread: Flow, nights: usize, options: &LeakOptions) -> Flow {
    #[allow(clippy::cast_precision_loss)] // small
    let n = nights.max(1) as f64;
    spread * (options.drift + options.threshold / n)
}

/// Watches `nights` for leaks.
///
/// # Errors
/// Fewer than `baseline_nights` + 1 present nights, fewer than 3 baseline
/// nights, options out of range, or no spread at all in the baseline.
///
/// # Panics
/// Only if an internal invariant is broken (no baseline formed although
/// enough nights were checked for) - that would be a bug.
pub fn watch(nights: &[Night], options: &LeakOptions) -> Result<Watch, BalanceError> {
    if options.baseline_nights < 3
        || !(options.drift >= 0.0 && options.threshold > 0.0 && options.meter_bias >= 0.0)
    {
        return Err(BalanceError::InvalidInput(
            "at least 3 baseline nights, drift >= 0, threshold > 0, bias >= 0".into(),
        ));
    }
    let present: Vec<usize> = (0..nights.len())
        .filter(|&k| nights[k].flow.is_finite())
        .collect();
    if present.len() <= options.baseline_nights {
        return Err(BalanceError::InvalidInput(format!(
            "{} night(s) with data: the baseline needs {} and the watch more",
            present.len(),
            options.baseline_nights
        )));
    }
    let mut cusum = vec![f64::NAN; nights.len()];
    let mut alarms = Vec::new();
    let mut from = 0; // index into `present` where the current baseline starts
    let mut first_stats = None;
    while from + options.baseline_nights < present.len() {
        let base = &present[from..from + options.baseline_nights];
        let (mu, s) = mean_and_spread(base.iter().map(|&k| nights[k].flow));
        if s.is_nan() || s <= 0.0 {
            return Err(BalanceError::InvalidInput(
                "the baseline nights do not vary at all: no spread to judge by".into(),
            ));
        }
        first_stats.get_or_insert((mu, s));
        let mut sum: f64 = 0.0;
        let mut left_zero = from + options.baseline_nights; // index into `present`
        let mut raised = None;
        for (j, &k) in present
            .iter()
            .enumerate()
            .skip(from + options.baseline_nights)
        {
            let z = (nights[k].flow - mu) / s;
            let next = (sum + z - options.drift).max(0.0);
            if sum <= 0.0 && next > 0.0 {
                left_zero = j;
            }
            sum = next;
            cusum[k] = sum;
            if sum > options.threshold {
                raised = Some(j);
                break;
            }
        }
        let Some(j) = raised else { break };
        let risen: Vec<f64> = present[left_zero..=j]
            .iter()
            .map(|&k| nights[k].flow)
            .collect();
        #[allow(clippy::cast_precision_loss)] // small counts
        let (m, b) = (risen.len() as f64, options.baseline_nights as f64);
        let rise = risen.iter().sum::<f64>() / m - mu;
        let random = s * (1.0 / m + 1.0 / b).sqrt();
        let bias = options.meter_bias * rise.abs();
        alarms.push(LeakAlarm {
            first_night: nights[present[left_zero]].start,
            raised: nights[present[j]].start,
            size: Flow::new(rise),
            uncertainty: Flow::new(random.hypot(bias)),
        });
        // Watch on from the alarm with a fresh baseline.
        from = j + 1;
    }
    let (mu, s) = first_stats.expect("at least one baseline was formed");
    Ok(Watch {
        baseline: Flow::new(mu),
        spread: Flow::new(s),
        smallest_in_a_week: smallest_detectable(Flow::new(s), 7, options),
        alarms,
        cusum,
    })
}

/// Mean and sample standard deviation.
fn mean_and_spread(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let v: Vec<f64> = values.collect();
    #[allow(clippy::cast_precision_loss)] // small counts
    let n = v.len() as f64;
    let mean = v.iter().sum::<f64>() / n;
    let var = v.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
    (mean, var.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nights(flows: &[f64]) -> Vec<Night> {
        flows
            .iter()
            .enumerate()
            .map(|(k, &flow)| Night {
                #[allow(clippy::cast_precision_loss)]
                start: Time::new(86_400.0 * k as f64),
                flow,
                readings: 8,
            })
            .collect()
    }

    /// The classic small settings, so the arithmetic below is easy.
    fn classic() -> LeakOptions {
        LeakOptions {
            baseline_nights: 14,
            threshold: 5.0,
            ..LeakOptions::default()
        }
    }

    #[test]
    fn a_step_of_three_spreads_is_caught_and_dated() {
        // Baseline alternates 1.0 +/- 0.1 (spread ~0.1036); from night 20
        // the flow is 0.3 higher: z ~ 2.9, CUSUM +2.4 a night: alarm on
        // the third risen night.
        let mut f: Vec<f64> = (0..20)
            .map(|k| if k % 2 == 0 { 1.1 } else { 0.9 })
            .collect();
        f.extend(std::iter::repeat_n(1.3, 10));
        let w = watch(&nights(&f), &classic()).unwrap();
        assert_eq!(w.alarms.len(), 1);
        let a = w.alarms[0];
        assert!((a.first_night.value() - 86_400.0 * 20.0).abs() < 1e-6);
        assert!((a.raised.value() - 86_400.0 * 22.0).abs() < 1e-6);
        assert!((a.size.value() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn missing_nights_neither_raise_nor_lower_the_watch() {
        let mut f: Vec<f64> = (0..20)
            .map(|k| if k % 2 == 0 { 1.1 } else { 0.9 })
            .collect();
        f.extend([f64::NAN, f64::NAN, 1.0, f64::NAN]);
        let w = watch(&nights(&f), &classic()).unwrap();
        assert!(w.alarms.is_empty());
        assert!(w.cusum[20].is_nan() && w.cusum[23].is_nan());
    }

    #[test]
    fn the_smallest_detectable_leak_follows_the_cusum_arithmetic() {
        let d = smallest_detectable(Flow::new(1e-3), 7, &classic());
        assert!((d.value() - 1e-3 * (0.5 + 5.0 / 7.0)).abs() < 1e-15);
    }
}
