//! The detection chain over one sensor's series of samples.
//!
//! 1. Calibrate: learn the sensor's noise (robust, from differences) and
//!    its starting level from a quiet opening period.
//! 2. Watch: each sample's distance from the baseline forecast, in noise
//!    units, feeds a two-sided CUSUM. The baseline (Brown's double
//!    smoothing, [`crate::baseline`]) follows demand drift - level and
//!    slope - over seconds, but not a transient (milliseconds).
//! 3. On an alarm: pick the arrival time with the step-area method, record
//!    the event, then hold off briefly (the transient rings on) and re-arm
//!    at the new level, keeping the drift trend learned so far.

use jk_kernel::{Head, Time};

use crate::baseline::Baseline;
pub use crate::cusum::Direction;
use crate::cusum::{Cusum, CusumConfig};
use crate::error::{DetectionError, positive};
use crate::noise::{median, sigma_from_differences};
use crate::onset::{self, OnsetConfig};

/// One sensor's samples. Sample k is the average over the interval ending
/// at `start + k * interval` (how integrating loggers report).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Series<'a> {
    /// Pressure head samples [m].
    pub samples: &'a [f64],
    /// Time of sample 0.
    pub start: Time,
    /// Sampling interval (1/128 s for common transient loggers).
    pub interval: Time,
}

impl Series<'_> {
    /// Time of (fractional) sample index `k`.
    #[must_use]
    pub fn time_at(&self, k: f64) -> Time {
        self.start + self.interval * k
    }
}

/// Settings of the whole chain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectorConfig {
    /// Quiet opening period used to learn noise and level.
    pub calibration: Time,
    /// CUSUM drift and threshold (noise units).
    pub cusum: CusumConfig,
    /// Memory of the baseline: it follows changes slower than this (drift)
    /// and stands firm against faster ones (transients).
    pub baseline_time_constant: Time,
    /// After an event, how long before watching again.
    pub hold_off: Time,
    /// Windows for arrival-time picking.
    pub onset: OnsetConfig,
    /// Floor on the noise estimate [m] (a perfectly quiet or heavily
    /// quantised signal must not make every tiny wiggle "infinitely"
    /// significant).
    pub min_noise: Head,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            calibration: Time::new(2.0),
            cusum: CusumConfig::default(),
            baseline_time_constant: Time::new(2.0),
            hold_off: Time::new(1.0),
            onset: OnsetConfig::default(),
            min_noise: Head::new(1e-3),
        }
    }
}

/// A detected transient at one sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Event {
    /// When the front arrived (sub-sample precision).
    pub arrival: Time,
    /// One-standard-deviation uncertainty of `arrival`.
    pub arrival_uncertainty: Time,
    /// When the alarm was raised (arrival + detection delay).
    pub alarm: Time,
    /// Size of the step: new level minus old level.
    pub step: Head,
    /// Rise or drop.
    pub direction: Direction,
}

/// The configured detection chain.
#[derive(Debug, Clone, PartialEq)]
pub struct Detector {
    config: DetectorConfig,
}

/// What a run found, and the noise it measured.
#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    /// Events in order of arrival.
    pub events: Vec<Event>,
    /// The sensor noise learned during calibration.
    pub noise: Head,
}

impl Detector {
    /// A detector with the given settings.
    ///
    /// # Errors
    /// Non-positive times, noise floor, or CUSUM settings.
    pub fn new(config: DetectorConfig) -> Result<Self, DetectionError> {
        positive("calibration", config.calibration.value())?;
        positive(
            "baseline_time_constant",
            config.baseline_time_constant.value(),
        )?;
        positive("hold_off", config.hold_off.value())?;
        positive("min_noise", config.min_noise.value())?;
        Cusum::new(config.cusum, 0)?;
        Ok(Self { config })
    }

    /// Runs the chain over `series`.
    ///
    /// # Errors
    /// A non-positive interval, a non-finite sample, or a series too short
    /// to calibrate on.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // ceil of positive ratios
    pub fn run(&self, series: &Series<'_>) -> Result<Detection, DetectionError> {
        let x = series.samples;
        let dt = series.interval.value();
        positive("sampling interval", dt)?;
        if let Some(index) = x.iter().position(|v| !v.is_finite()) {
            return Err(DetectionError::NonFiniteSample { index });
        }
        let calibration = ((self.config.calibration.value() / dt).ceil() as usize).max(16);
        if x.len() <= calibration + self.config.onset.after + 1 {
            return Err(DetectionError::TooShort {
                samples: x.len(),
                needed: calibration + self.config.onset.after + 2,
            });
        }
        let hold_off = ((self.config.hold_off.value() / dt).ceil() as usize).max(1);
        let sigma = sigma_from_differences(&x[..calibration]).max(self.config.min_noise.value());
        let alpha = (dt / self.config.baseline_time_constant.value()).min(1.0);

        // Warm the baseline up over the calibration window, so it has
        // already learned the drift trend when watching starts.
        let mut baseline = Baseline::new(median(&mut x[..calibration.min(16)].to_vec()), alpha);
        for &v in &x[..calibration] {
            baseline.update(v);
        }
        let mut cusum = Cusum::new(self.config.cusum, calibration)?;
        let mut events = Vec::new();
        let mut k = calibration;
        while k < x.len() {
            let Some(alarm) = cusum.push((x[k] - baseline.forecast()) / sigma) else {
                baseline.update(x[k]);
                k += 1;
                continue;
            };
            if let Some(o) = onset::pick(
                x,
                alarm.change_start,
                alarm.index,
                &self.config.onset,
                sigma,
            ) {
                events.push(Event {
                    arrival: series.time_at(o.index),
                    arrival_uncertainty: series.interval * o.uncertainty,
                    #[allow(clippy::cast_precision_loss)]
                    alarm: series.time_at(alarm.index as f64),
                    step: Head::new(o.post - o.pre),
                    direction: alarm.direction,
                });
            }
            // Hold off while the transient rings, then re-arm at the level
            // the signal has settled to, keeping the drift trend.
            let rearm = (alarm.index + hold_off).min(x.len());
            if rearm >= x.len() {
                break;
            }
            let recent = rearm.saturating_sub(self.config.onset.after.max(4));
            let settled = median(&mut x[recent..rearm].to_vec());
            // Move the forecast to the settled level; the trend is kept.
            let expected_now = baseline.forecast();
            baseline.shift(settled - expected_now);
            cusum.reset_at(rearm);
            k = rearm;
        }
        Ok(Detection {
            events,
            noise: Head::new(sigma),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn series(samples: &[f64]) -> Series<'_> {
        Series {
            samples,
            start: Time::ZERO,
            interval: Time::new(1.0 / 128.0),
        }
    }

    #[test]
    fn finds_one_clean_drop_and_times_it() {
        // 4 s quiet at 60 m, then a sharp 4.5 m drop at t = 3.0 s exactly
        // (sample 384), then quiet again.
        let x: Vec<f64> = (0..768)
            .map(|k| if k < 384 { 60.0 } else { 55.5 })
            .collect();
        let d = Detector::new(DetectorConfig::default())
            .unwrap()
            .run(&series(&x))
            .unwrap();
        assert_eq!(d.events.len(), 1);
        let e = d.events[0];
        assert_eq!(e.direction, Direction::Drop);
        assert!((e.step.value() + 4.5).abs() < 1e-12);
        // Sample 384 is the first fully-dropped sample: the front came at
        // the start of its interval, t = 383/128.
        assert!(
            (e.arrival.value() - 383.0 / 128.0).abs() < 1e-12,
            "{}",
            e.arrival
        );
    }

    #[test]
    fn refuses_gaps_and_short_series() {
        let det = Detector::new(DetectorConfig::default()).unwrap();
        let mut x = vec![60.0; 600];
        x[300] = f64::NAN;
        assert!(matches!(
            det.run(&series(&x)),
            Err(DetectionError::NonFiniteSample { index: 300 })
        ));
        assert!(matches!(
            det.run(&series(&[60.0; 100])),
            Err(DetectionError::TooShort { .. })
        ));
    }
}
