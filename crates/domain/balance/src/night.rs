//! One night flow per night: the mean balance over the night window.
//!
//! Night windows are in LOCAL time (the zone's offset from UTC; South
//! Africa: +2 h), default 02:00 to 04:00. A night with less than half of
//! its readings present is missing - reported, not filled (P6).

use jk_kernel::Time;

use crate::BalanceError;
use crate::zone::Series;

const DAY: f64 = 86_400.0;

/// When nights are measured.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NightWindow {
    /// Local time minus UTC (South Africa: 2 hours).
    pub utc_offset: Time,
    /// Start of the window, time after local midnight.
    pub from: Time,
    /// End of the window, time after local midnight.
    pub to: Time,
}

impl Default for NightWindow {
    fn default() -> Self {
        Self {
            utc_offset: Time::new(7200.0),
            from: Time::new(2.0 * 3600.0),
            to: Time::new(4.0 * 3600.0),
        }
    }
}

/// One night's flow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Night {
    /// Start of the night's window (UTC, as the series).
    pub start: Time,
    /// Mean balance over the window [m^3/s]; NaN if the night is missing.
    pub flow: f64,
    /// Readings used.
    pub readings: usize,
}

/// The night flow of every night the series covers (whole windows only).
///
/// # Errors
/// A window that does not fit in a day or ends before it starts.
pub fn night_flows(series: &Series, window: NightWindow) -> Result<Vec<Night>, BalanceError> {
    let (from, to) = (window.from.value(), window.to.value());
    if !(0.0 <= from && from < to && to <= DAY) {
        return Err(BalanceError::InvalidInput(format!(
            "night window {from} s to {to} s after midnight"
        )));
    }
    let dt = series.interval.value();
    let t0 = series.start.value();
    let n = series.values.len();
    #[allow(clippy::cast_precision_loss)] // counts are small
    let end = t0 + dt * n as f64;
    let offset = window.utc_offset.value();
    // First local midnight at or before the start.
    let mut midnight = ((t0 + offset) / DAY).floor() * DAY - offset;
    let mut out = Vec::new();
    while midnight + from < end {
        let (a, b) = (midnight + from, midnight + to);
        if a >= t0 && b <= end {
            // Readings whose time lies in [a, b).
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // in range
            let (first, last) = (
                ((a - t0) / dt - 1e-9).ceil().max(0.0) as usize,
                (((b - t0) / dt - 1e-9).ceil() as usize).min(n),
            );
            let present: Vec<f64> = series.values[first..last]
                .iter()
                .copied()
                .filter(|v| v.is_finite())
                .collect();
            let expected = last - first;
            #[allow(clippy::cast_precision_loss)] // counts are small
            let flow = if expected > 0 && 2 * present.len() >= expected {
                present.iter().sum::<f64>() / present.len() as f64
            } else {
                f64::NAN
            };
            out.push(Night {
                start: Time::new(a),
                flow,
                readings: present.len(),
            });
        }
        midnight += DAY;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nights_are_the_mean_over_the_local_window() {
        // Two days of 15-minute readings from local midnight (UTC 22:00
        // the day before, SAST = UTC + 2): value = the hour of the day.
        let start = 1_790_632_800.0; // 22:00 UTC = local midnight (SAST)
        let values: Vec<f64> = (0..192).map(|k| f64::from((k / 4) % 24)).collect();
        let s = Series {
            start: Time::new(start),
            interval: Time::new(900.0),
            values,
        };
        let nights = night_flows(&s, NightWindow::default()).unwrap();
        assert_eq!(nights.len(), 2);
        // 02:00-04:00: hours 2 and 3, four readings each: mean 2.5.
        assert!((nights[0].flow - 2.5).abs() < 1e-12);
        assert_eq!(nights[0].readings, 8);
    }

    #[test]
    fn a_night_mostly_missing_is_missing() {
        let mut values = vec![1.0; 96];
        for v in &mut values[8..14] {
            *v = f64::NAN; // 6 of the 8 night readings gone
        }
        let s = Series {
            start: Time::new(-7200.0),
            interval: Time::new(900.0),
            values,
        };
        let nights = night_flows(&s, NightWindow::default()).unwrap();
        assert!(nights[0].flow.is_nan());
    }
}
