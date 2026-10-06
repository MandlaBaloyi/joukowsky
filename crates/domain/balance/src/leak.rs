//! A leak alarm: a lasting rise in night flow beyond how much nights vary.
//!
//! The baseline is the first `baseline_nights` present nights: their mean
//! mu and night-to-night spread s - and, when those weeks show one, a
//! WEEKLY PROFILE: each weekday's own offset from mu (weekend nights can
//! use far more than weeknights - L-Town's 10-25 m3/h, FN-048). From then
//! on each night is judged against what is expected of it, by a two-sided
//! CUSUM (Page 1954):
//!
//! ```text
//!   expected e_k = mu + w_d (d: the night's weekday);  z_k = (flow - e_k) / s
//!   S+_k = max(0, S+_{k-1} + z_k - drift)    a LEAK when S+ > threshold
//!   S-_k = max(0, S-_{k-1} - z_k - drift)    a FALL when S- > threshold
//! ```
//!
//! Each night counts at most `cap` spreads either way (a robust CUSUM): a
//! leak shows itself night after night, while one wild night - L-Town's
//! odd Sunday ran 20-30 m3/h over even its own weekday's level - must not
//! sound an alarm almost alone (31 of 59 alarms were false without it,
//! nearly all raised on a Monday, FN-048).
//!
//! A leak alarm dates the leak to the night after S+ last stood at zero and
//! sizes it as the mean excess since then, with its standard error (P3). A
//! fall is the same downwards: a leak repaired, or less use. Either way
//! the expected level then STEPS by the size found, and for
//! `settle_nights` it is refined as the MEDIAN of every night since the
//! change began: a size measured BECAUSE it crossed the alarm line is
//! biased high, and stepping by it alone made the watch ping-pong between
//! alarms and falls (one false alarm per 250 nights in simulation,
//! FN-048); the median, not the mean, so that a second leak beginning
//! inside the window is not swallowed into the new level (the mean
//! swallowed three of L-Town's big leaks, FN-051).
//! Then the watch goes on - it does not stop four weeks to learn a new
//! baseline (that baseline held the leak, its repair and the season, and
//! went blind for 341 days on L-Town, FN-048). While nothing builds up (both sums at zero) what is
//! expected follows slow change - the seasons - over `follow_nights`; with
//! a weekly profile each weekday follows its own nights (over
//! `follow_nights` / 7 of them: four weeks by default), so the profile
//! keeps learning - four winter weeks are not the whole year (L-Town's
//! Sunday nights looked ordinary in January and run 15 m3/h higher over
//! the year).
//!
//! Missing nights are skipped - they neither raise nor lower anything
//! (P6) - and so are holiday nights the site's calendar marks
//! ([`crate::night::mark_holidays`], G-24). A meter's calibration error shifts every night alike: it cancels
//! here and only blurs the ABSOLUTE size (reported as the size's
//! uncertainty from `meter_bias`).

use jk_kernel::{Flow, Time};

use crate::BalanceError;
use crate::night::Night;

/// The watch after a change: what was expected before it, the size the
/// alarm stepped by, every night (index, excess over `before`) since the
/// change began, and the nights left.
type Settling = ([f64; 7], f64, Vec<(usize, f64)>, usize);

/// How clear a second change inside a settling window must be: its size
/// over the standard error of the difference of the means either side.
const SPLIT_SIGMAS: f64 = 5.0;

/// F at the 1% level for 6 and 21 degrees of freedom (seven weekdays, 28
/// nights): the weekday means of a baseline differ by more than chance
/// when the between-weekday variance exceeds the within-weekday variance
/// by this factor (F tables; 3.81).
const F_WEEKLY_1PCT: f64 = 3.81;
/// Fewest baseline nights for a weekly profile: four of each weekday.
const WEEKLY_NIGHTS: usize = 28;

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
    /// Learn a weekly profile when the baseline shows one (an F test at the
    /// 1% level, at least 28 baseline nights). Default true.
    pub weekly: bool,
    /// The most one night counts in the CUSUM, in spreads (z is clipped
    /// to +/- cap). Default 3 [VERIFY on more zones]: below it nothing
    /// changes; a leak of more than 3.5 spreads is caught a night or two
    /// later than without the cap.
    pub cap: f64,
    /// Nights after an alarm or a fall during which the new level is
    /// refined (the mean of the nights since the change began) and no new
    /// alarm is raised. Default 14.
    pub settle_nights: usize,
    /// Nights over which the expected level follows slow change (the
    /// seasons) while nothing builds up; 0: it never moves on its own.
    /// Default 28 [VERIFY on more zones]: L-Town's night flow climbs about
    /// 19 m3/h from January to July (FN-048), far slower than a leak.
    pub follow_nights: f64,
}

impl Default for LeakOptions {
    fn default() -> Self {
        Self {
            baseline_nights: 28,
            drift: 0.5,
            threshold: 6.0,
            meter_bias: 0.02,
            cap: 3.0,
            settle_nights: 14,
            weekly: true,
            follow_nights: 28.0,
        }
    }
}

/// A leak found (or a fall: the same, downwards).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeakAlarm {
    /// The first night of the change (as dated by the CUSUM).
    pub first_night: Time,
    /// The night the alarm was raised.
    pub raised: Time,
    /// Estimated change: mean night flow since the first night minus what
    /// was expected (negative for a fall).
    pub size: Flow,
    /// One-sigma uncertainty of the size: the spread over the nights
    /// averaged, the baseline's own uncertainty, and the meters' possible
    /// calibration error.
    pub uncertainty: Flow,
}

/// What the watch found over the nights given.
#[derive(Debug, Clone, PartialEq)]
pub struct Watch {
    /// Baseline night flow mu [m^3/s] (the mean of the baseline nights).
    pub baseline: Flow,
    /// Night-to-night spread s [m^3/s], once the weekly profile is taken
    /// out.
    pub spread: Flow,
    /// Each weekday's offset from the baseline [m^3/s], Monday first, when
    /// the baseline showed a weekly profile.
    pub weekly: Option<[f64; 7]>,
    /// The smallest leak caught within a week, with this spread
    /// ([`smallest_detectable`]).
    pub smallest_in_a_week: Flow,
    /// Leak alarms, in order.
    pub alarms: Vec<LeakAlarm>,
    /// Lasting falls - a leak repaired, or less use - in order.
    pub falls: Vec<LeakAlarm>,
    /// The upward CUSUM after each night (NaN for missing or baseline
    /// nights).
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

/// The baseline: mean, spread, and the weekly profile if the nights show
/// one.
fn learn(base: &[Night], options: &LeakOptions) -> (f64, f64, Option<[f64; 7]>) {
    let (mu, s) = mean_and_spread(base.iter().map(|n| n.flow));
    if !options.weekly || base.len() < WEEKLY_NIGHTS {
        return (mu, s, None);
    }
    let mut sum = [0.0; 7];
    let mut count = [0usize; 7];
    for n in base {
        let d = usize::from(n.weekday % 7);
        sum[d] += n.flow;
        count[d] += 1;
    }
    if count.iter().any(|&c| c < 2) {
        return (mu, s, None);
    }
    #[allow(clippy::cast_precision_loss)] // small counts
    let means: Vec<f64> = (0..7).map(|d| sum[d] / count[d] as f64).collect();
    #[allow(clippy::cast_precision_loss)] // small counts
    let between: f64 = (0..7)
        .map(|d| count[d] as f64 * (means[d] - mu).powi(2))
        .sum();
    let within: f64 = base
        .iter()
        .map(|n| (n.flow - means[usize::from(n.weekday % 7)]).powi(2))
        .sum();
    #[allow(clippy::cast_precision_loss)] // small counts
    let dof = (base.len() - 7) as f64;
    let f = (between / 6.0) / (within / dof);
    if f.is_nan() || f <= F_WEEKLY_1PCT {
        return (mu, s, None);
    }
    let mut offsets = [0.0; 7];
    for d in 0..7 {
        offsets[d] = means[d] - mu;
    }
    (mu, (within / dof).sqrt(), Some(offsets))
}

/// Watches `nights` for leaks (and falls).
#[allow(clippy::too_many_lines)] // one pass over the nights, read top to bottom
///
/// # Errors
/// Fewer than `baseline_nights` + 1 present nights, fewer than 3 baseline
/// nights, options out of range, or no spread at all in the baseline.
pub fn watch(nights: &[Night], options: &LeakOptions) -> Result<Watch, BalanceError> {
    if options.baseline_nights < 3
        || !(options.drift >= 0.0
            && options.threshold > 0.0
            && options.meter_bias >= 0.0
            && options.cap > options.drift
            && options.follow_nights >= 0.0)
    {
        return Err(BalanceError::InvalidInput(
            "at least 3 baseline nights, drift >= 0, threshold > 0, bias >= 0, cap > drift,              follow >= 0"
                .into(),
        ));
    }
    // Nights judged: with data, and not holidays.
    let present: Vec<usize> = (0..nights.len())
        .filter(|&k| nights[k].flow.is_finite() && !nights[k].holiday)
        .collect();
    if present.len() <= options.baseline_nights {
        return Err(BalanceError::InvalidInput(format!(
            "{} night(s) with data: the baseline needs {} and the watch more",
            present.len(),
            options.baseline_nights
        )));
    }
    let base: Vec<Night> = present[..options.baseline_nights]
        .iter()
        .map(|&k| nights[k])
        .collect();
    let (mu0, s, weekly) = learn(&base, options);
    if s.is_nan() || s <= 0.0 {
        return Err(BalanceError::InvalidInput(
            "the baseline nights do not vary at all: no spread to judge by".into(),
        ));
    }
    // What is expected of each weekday's night.
    let mut expected = [mu0; 7];
    if let Some(w) = weekly {
        for (e, o) in expected.iter_mut().zip(w) {
            *e += o;
        }
    }
    let follow = if weekly.is_some() {
        options.follow_nights / 7.0
    } else {
        options.follow_nights
    }
    .max(1.0);
    #[allow(clippy::cast_precision_loss)] // small counts
    let b = options.baseline_nights as f64;
    let mut cusum = vec![f64::NAN; nights.len()];
    let (mut alarms, mut falls) = (Vec::new(), Vec::new());
    let (mut up, mut down) = (0.0_f64, 0.0_f64);
    // Excess of each watched night over what was expected of it, and where
    // each sum last left zero (indices into `excess`).
    let mut excess: Vec<(usize, f64)> = Vec::new();
    let (mut up_from, mut down_from) = (0, 0);
    // After a change: what was expected before it, the size the alarm
    // stepped by, every night (index, excess over `before`) since the change
    // began, and the nights left.
    let mut settling: Option<Settling> = None;
    let week = smallest_detectable(Flow::new(s), 7, options).value();
    for &k in &present[options.baseline_nights..] {
        let night = nights[k];
        let d = usize::from(night.weekday % 7);
        if let Some((before, stepped, seen, left)) = settling.as_mut() {
            seen.push((k, night.flow - before[d]));
            *left -= 1;
            let values: Vec<f64> = seen.iter().map(|(_, e)| *e).collect();
            let shift = median(&values);
            for (x, b) in expected.iter_mut().zip(before.iter()) {
                *x = b + shift;
            }
            cusum[k] = 0.0;
            if *left == 0 {
                // A second change inside the window - a new leak right after
                // the first, or the first repaired at once - shows as a
                // level clearly away from the step: report it too.
                if let Some((change, after)) =
                    second_change(seen, *stepped, week, s, b, nights, k, options)
                {
                    // The level is now what the window's last nights show
                    // (or the new leak would be found again, FN-051).
                    for (x, b) in expected.iter_mut().zip(before.iter()) {
                        *x = b + after;
                    }
                    if change.size.value() > 0.0 {
                        alarms.push(change);
                    } else {
                        falls.push(change);
                    }
                }
                settling = None;
            }
            continue;
        }
        let e = night.flow - expected[d];
        let z = (e / s).clamp(-options.cap, options.cap);
        let j = excess.len();
        excess.push((k, e));
        let next_up = (up + z - options.drift).max(0.0);
        let next_down = (down - z - options.drift).max(0.0);
        if up <= 0.0 && next_up > 0.0 {
            up_from = j;
        }
        if down <= 0.0 && next_down > 0.0 {
            down_from = j;
        }
        (up, down) = (next_up, next_down);
        cusum[k] = up;
        let changed = if up > options.threshold {
            Some((up_from, &mut alarms))
        } else if down > options.threshold {
            Some((down_from, &mut falls))
        } else {
            None
        };
        if let Some((from, list)) = changed {
            let since = &excess[from..=j];
            #[allow(clippy::cast_precision_loss)] // small counts
            let m = since.len() as f64;
            let size = since.iter().map(|(_, e)| e).sum::<f64>() / m;
            let random = s * (1.0 / m + 1.0 / b).sqrt();
            let bias = options.meter_bias * size.abs();
            list.push(LeakAlarm {
                first_night: nights[since[0].0].start,
                raised: night.start,
                size: Flow::new(size),
                uncertainty: Flow::new(random.hypot(bias)),
            });
            // The change is now part of what is expected; refine it, then
            // watch on.
            let before = expected;
            for x in &mut expected {
                *x += size;
            }
            if options.settle_nights > 0 {
                settling = Some((before, size, since.to_vec(), options.settle_nights));
            }
            (up, down) = (0.0, 0.0);
        } else if up <= 0.0 && down <= 0.0 && options.follow_nights > 0.0 {
            if weekly.is_some() {
                expected[d] += e / follow;
            } else {
                for x in &mut expected {
                    *x += e / follow;
                }
            }
        }
    }
    Ok(Watch {
        baseline: Flow::new(mu0),
        spread: Flow::new(s),
        weekly,
        smallest_in_a_week: smallest_detectable(Flow::new(s), 7, options),
        alarms,
        falls,
        cusum,
    })
}

/// At the end of a settling window: a second change inside it - a new
/// leak, or the first repaired at once - is a clear split of the window's
/// OWN nights (those after the alarm, at least 3 either side of the split)
/// into two levels, apart by more than a week's detectable leak and by
/// more than [`SPLIT_SIGMAS`] standard errors. The nights before the alarm
/// are left out: they hold the first leak's own start, and an alarm raised
/// early in a rise underestimates it - its remainder is not a new change
/// (FN-051). Dated at the split, sized as the difference; returned with
/// the level after the split (over what was expected before the alarm).
#[allow(clippy::too_many_arguments)] // the window's state, passed once
fn second_change(
    seen: &[(usize, f64)],
    stepped: f64,
    week: f64,
    s: f64,
    b: f64,
    nights: &[Night],
    raised_at: usize,
    options: &LeakOptions,
) -> Option<(LeakAlarm, f64)> {
    let _ = stepped;
    // The window's own nights: those after the alarm.
    let own = &seen[seen.len().saturating_sub(options.settle_nights)..];
    let values: Vec<f64> = own.iter().map(|(_, e)| *e).collect();
    let n = values.len();
    // The best split: before it the first change's level, after it the new.
    let mut best = (0.0_f64, 0usize);
    for cut in 3..n.saturating_sub(2) {
        #[allow(clippy::cast_precision_loss)] // small counts
        let (m1, m2) = (cut as f64, (n - cut) as f64);
        let before = values[..cut].iter().sum::<f64>() / m1;
        let after = values[cut..].iter().sum::<f64>() / m2;
        if (after - before).abs() > best.0.abs() {
            best = (after - before, cut);
        }
    }
    let (size, cut) = best;
    #[allow(clippy::cast_precision_loss)] // small counts
    let (m1, m) = (cut as f64, (n - cut) as f64);
    // A clear step, not the window's own scatter or a leak growing slowly:
    // SPLIT_SIGMAS standard errors of the difference of the two means.
    let clear = SPLIT_SIGMAS * s * (1.0 / m1 + 1.0 / m).sqrt();
    if cut == 0 || size.abs() <= week.max(clear) {
        return None;
    }
    let random = s * (1.0 / m + 1.0 / b).sqrt();
    let after = values[cut..].iter().sum::<f64>() / m;
    Some((
        LeakAlarm {
            first_night: nights[own[cut].0].start,
            raised: nights[raised_at].start,
            size: Flow::new(size),
            uncertainty: Flow::new(random.hypot(options.meter_bias * size.abs())),
        },
        after,
    ))
}

/// Median of `values` (NaN-free).
fn median(values: &[f64]) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    let n = v.len();
    if n == 0 {
        0.0
    } else if n % 2 == 1 {
        v[n / 2]
    } else {
        f64::midpoint(v[n / 2 - 1], v[n / 2])
    }
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
                #[allow(clippy::cast_possible_truncation)] // 0..7
                weekday: (k % 7) as u8,
                #[allow(clippy::cast_possible_wrap)] // small
                day: k as i64,
                holiday: false,
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
