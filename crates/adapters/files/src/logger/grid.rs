//! Timestamps to a regular grid, split where rows are missing.
//!
//! A logger samples at a fixed rate; its file's timestamps are rounded
//! (often to the millisecond: at 128 S/s that is a jitter of up to half a
//! millisecond) and rows can be missing (a lost upload), repeated, or out
//! of order. The engine wants what the logger actually did: samples every
//! `interval` from a start time, in continuous runs.
//!
//! 1. The interval: the MEAN of the ordinary steps between rows (those
//!    within half of the median step - gaps and repeats left out). Not the
//!    median: millisecond timestamps at 128 S/s step by 7 or 8 ms, so the
//!    median says 8 ms (125 S/s) while the mean says 7.8125 ms (measured
//!    by a test). If that is within 0.2% of a whole number of samples per
//!    second, and snapping to it keeps every row within 0.3 of a sample of
//!    its slot, the whole rate is used (the logger's nominal rate);
//!    otherwise the rate that fits the first and last rows.
//! 2. Every row takes the nearest slot k = round((t - t0) / interval).
//! 3. A slot after the previous one continues the run; a jump starts a new
//!    run (the rows in between never arrived - a gap, reported); a slot at
//!    or before the previous one is a repeated or out-of-order row: dropped
//!    and counted, never mixed in.

/// A continuous run of rows.
#[derive(Debug, Clone, PartialEq)]
pub struct Span {
    /// Time of its first slot [s].
    pub start: f64,
    /// Indices of its rows, one per consecutive slot.
    pub rows: Vec<usize>,
}

/// The rows placed on a regular grid.
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    /// Seconds between samples.
    pub interval: f64,
    /// Whether the clock is regular: a whole number of samples a second
    /// (transient loggers) or of seconds a sample (SCADA, 1 s loggers).
    pub whole_rate: bool,
    /// The continuous runs, in time order.
    pub spans: Vec<Span>,
    /// Rows dropped as repeated or out of order.
    pub dropped: Vec<usize>,
    /// Samples missing between the runs.
    pub missing: usize,
    /// Largest distance of a timestamp from its slot [s].
    pub jitter: f64,
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

#[allow(clippy::cast_possible_truncation)] // slot numbers are small
fn slot(t: f64, t0: f64, dt: f64) -> i64 {
    ((t - t0) / dt).round() as i64
}

#[allow(clippy::cast_precision_loss)] // slot numbers are far below 2^52
fn at(t0: f64, dt: f64, k: i64) -> f64 {
    t0 + dt * k as f64
}

/// Places `times` (in file order) on a regular grid. None if fewer than two
/// increasing timestamps.
#[must_use]
pub fn regularise(times: &[f64]) -> Option<Grid> {
    let steps: Vec<f64> = times
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|d| *d > 0.0)
        .collect();
    if steps.is_empty() {
        return None;
    }
    let t0 = times[0];
    let typical = median(steps.clone());
    let ordinary: Vec<f64> = steps
        .into_iter()
        .filter(|d| (d - typical).abs() < 0.5 * typical)
        .collect();
    #[allow(clippy::cast_precision_loss)] // counts are far below 2^52
    let raw = ordinary.iter().sum::<f64>() / ordinary.len() as f64;
    let worst = |dt: f64| {
        times
            .iter()
            .map(|&t| (t - at(t0, dt, slot(t, t0, dt))).abs() / dt)
            .fold(0.0, f64::max)
    };
    let rate = 1.0 / raw;
    let whole = rate.round();
    let seconds = raw.round();
    let (interval, whole_rate) =
        if whole >= 1.0 && ((rate - whole) / rate).abs() < 2e-3 && worst(1.0 / whole) < 0.3 {
            (1.0 / whole, true)
        } else if seconds >= 1.0 && ((raw - seconds) / raw).abs() < 2e-3 && worst(seconds) < 0.3 {
            (seconds, true)
        } else if worst(raw) < 0.3 {
            (raw, false)
        } else {
            // A drifting clock: fit the slots of the first and last rows.
            let last = *times.last().unwrap_or(&t0);
            let k = slot(last, t0, raw).max(1);
            #[allow(clippy::cast_precision_loss)]
            let fit = (last - t0) / k as f64;
            (fit, false)
        };
    let (mut spans, mut dropped, mut missing, mut jitter) =
        (Vec::<Span>::new(), Vec::new(), 0usize, 0.0_f64);
    let mut prev: Option<i64> = None;
    for (i, &t) in times.iter().enumerate() {
        let k = slot(t, t0, interval);
        jitter = jitter.max((t - at(t0, interval, k)).abs());
        match prev {
            Some(p) if k <= p => dropped.push(i),
            Some(p) if k == p + 1 => {
                if let Some(s) = spans.last_mut() {
                    s.rows.push(i);
                }
                prev = Some(k);
            }
            _ => {
                if let Some(p) = prev {
                    missing += usize::try_from(k - p - 1).unwrap_or(0);
                }
                spans.push(Span {
                    start: at(t0, interval, k),
                    rows: vec![i],
                });
                prev = Some(k);
            }
        }
    }
    Some(Grid {
        interval,
        whole_rate,
        spans,
        dropped,
        missing,
        jitter,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scada_every_five_minutes_is_a_regular_clock() {
        // L-Town's SCADA (BattLeDIM 2020): one row every 300 s.
        let times: Vec<f64> = (0..100).map(|k| f64::from(k) * 300.0).collect();
        let g = regularise(&times).unwrap();
        assert!(g.whole_rate && (g.interval - 300.0).abs() < 1e-12);
    }

    #[test]
    fn millisecond_timestamps_at_128_per_second_snap_to_128() {
        // Timestamps rounded to 1 ms, as loggers write them.
        let times: Vec<f64> = (0..1000)
            .map(|k| (f64::from(k) / 128.0 * 1000.0).round() / 1000.0)
            .collect();
        let g = regularise(&times).unwrap();
        assert!(g.whole_rate && (g.interval - 1.0 / 128.0).abs() < 1e-15);
        assert_eq!(g.spans.len(), 1);
        assert!(g.jitter <= 0.0005 + 1e-12, "{}", g.jitter);
    }

    #[test]
    fn a_missing_second_splits_the_run_and_repeats_are_dropped() {
        let mut times: Vec<f64> = (0..10).map(f64::from).collect(); // 1 S/s
        times.retain(|t| (*t - 4.0).abs() > 0.5 && (*t - 5.0).abs() > 0.5);
        times.insert(3, 2.0); // a repeated row
        let g = regularise(&times).unwrap();
        assert_eq!(g.spans.len(), 2);
        assert_eq!(g.missing, 2);
        assert_eq!(g.dropped, vec![3]);
        assert!((g.spans[1].start - 6.0).abs() < 1e-12);
    }
}
