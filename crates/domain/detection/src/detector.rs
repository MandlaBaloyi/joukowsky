//! The detection chain over one sensor's series of samples.
//!
//! 1. Calibrate: learn the sensor's noise (robust, from differences) and
//!    its starting level from a quiet opening period.
//! 2. Watch: each sample's distance from the baseline forecast, in noise
//!    units, feeds a two-sided CUSUM. The baseline (Brown's double
//!    smoothing, [`crate::baseline`]) follows demand drift - level and
//!    slope - over seconds, but not a transient (milliseconds).
//! 3. On an alarm: pick the arrival time with the step-area method and
//!    record the event. After an event the network RINGS - the wave echoes
//!    off reservoirs, tees and dead ends - and each echo would otherwise be
//!    taken for a new event (measured: one burst produced 11 extra alerts).
//!    So the sensor is blind for the hold-off and one settle window (the
//!    pipeline makes that at least one echo round trip of the network),
//!    then listens again at the new level, keeping the drift trend:
//!    - if its recent record is quiet (spread back near the noise), at full
//!      sensitivity;
//!    - if the network still rings, at REDUCED sensitivity: the noise unit
//!      is raised so the largest recent swing counts as `ring_units` noise
//!      units - below the CUSUM drift, so the ringing stays silent while a
//!      new event about twice its size still fires. The unit is re-measured
//!      every quarter window as the ringing dies away.
//!
//!    Waiting blind until the network is quiet was tried first and failed:
//!    a flowing network rang for over 25 s and a second burst went unseen.
//!    How the sensor re-armed is recorded on each event (principle P2).

use jk_kernel::{Head, Time};

use crate::baseline::Baseline;
pub use crate::change::Direction;
use crate::change::{Cusum, CusumConfig};
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
    /// After an event, the least time before watching again.
    pub hold_off: Time,
    /// Length of record judged for quiet (and blind after an event).
    pub settle_window: Time,
    /// "Quiet": every sample in the settle window within this many noise
    /// units of the window's median - what plain noise looks like. (Over n
    /// samples Gaussian noise peaks near sqrt(2 ln n) units: 3.4 for 300.)
    /// Any looser and leftover ringing re-triggers the full-sensitivity
    /// CUSUM, which fires on sustained shifts above its drift, 3 units.
    pub settle_band: f64,
    /// While the network still rings, its largest recent swing counts as
    /// this many noise units. Keep it below the CUSUM drift.
    pub ring_units: f64,
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
            settle_window: Time::new(0.5),
            settle_band: 4.0,
            ring_units: 1.0,
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
    /// How the sensor came back to watching after this event.
    pub rearm: Rearm,
}

/// How a sensor came back to watching after an event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Rearm {
    /// Quiet again; watching at full sensitivity from this time.
    Settled(Time),
    /// Watching again from `from` while the network still rang, with the
    /// noise unit raised to `floor`: events smaller than a few `floor`s
    /// could not be seen until the ringing died away.
    Ringing {
        /// When watching resumed.
        from: Time,
        /// The raised noise unit at that time.
        floor: Head,
    },
    /// The record ended before the sensor could watch again.
    RecordEnded,
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
    /// When the sensor was blind or listening at reduced sensitivity (after
    /// its own events, while the network rang). Outside these spans it
    /// watched at full sensitivity: its silence there means "nothing
    /// happened", inside them it does not (principles P2, P6).
    pub impaired: Vec<Span>,
}

/// A stretch of time, inclusive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    /// Start.
    pub from: Time,
    /// End.
    pub to: Time,
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
        positive("settle_window", config.settle_window.value())?;
        positive("settle_band", config.settle_band)?;
        positive("ring_units", config.ring_units)?;
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
        let settle = ((self.config.settle_window.value() / dt).ceil() as usize).max(4);
        let sigma = sigma_from_differences(&x[..calibration]).max(self.config.min_noise.value());
        let alpha = (dt / self.config.baseline_time_constant.value()).min(1.0);

        // Warm the baseline up over the calibration window, so it has
        // already learned the drift trend when watching starts.
        let mut baseline = Baseline::new(median(&mut x[..calibration.min(16)].to_vec()), alpha);
        for &v in &x[..calibration] {
            baseline.update(v);
        }
        let mut cusum = Cusum::new(self.config.cusum, calibration)?;
        let mut floor = Floor::new(*series, sigma, settle, &self.config);
        let mut events = Vec::new();
        let mut k = calibration;
        while k < x.len() {
            if k >= floor.next_check {
                let (centre, quiet) = floor.measure(k, true);
                if !quiet {
                    // Judge the next samples against the ringing's centre,
                    // the same centre its size was measured around.
                    let expected_now = baseline.forecast();
                    baseline.shift(centre - expected_now);
                }
            }
            let Some(alarm) = cusum.push((x[k] - baseline.forecast()) / floor.scale) else {
                baseline.update(x[k]);
                k += 1;
                continue;
            };
            let Some(o) = onset::pick(
                x,
                alarm.change_start,
                alarm.index,
                &self.config.onset,
                floor.scale,
            ) else {
                // A swing with no step behind it: ringing that crossed the
                // floor. Raise the floor to it and keep listening.
                floor.measure((k + 1).max(settle), false);
                baseline.update(x[k]);
                cusum.reset_at(k + 1);
                k += 1;
                continue;
            };
            floor.impaired_from(alarm.index);
            // Blind for the hold-off and one settle window, then listen again.
            let at = alarm.index + hold_off.max(settle);
            let rearm = if at < x.len() {
                let (centre, quiet) = floor.measure(at, true);
                // Move the forecast to the new level; the trend is kept.
                let expected_now = baseline.forecast();
                baseline.shift(centre - expected_now);
                cusum.reset_at(at);
                let from = series.time_at(to_f64(at));
                if quiet {
                    Rearm::Settled(from)
                } else {
                    Rearm::Ringing {
                        from,
                        floor: Head::new(floor.scale),
                    }
                }
            } else {
                Rearm::RecordEnded
            };
            events.push(Event {
                arrival: series.time_at(o.index),
                arrival_uncertainty: series.interval * o.uncertainty,
                alarm: series.time_at(to_f64(alarm.index)),
                step: Head::new(o.post - o.pre),
                direction: alarm.direction,
                rearm,
            });
            k = at;
        }
        Ok(Detection {
            events,
            noise: Head::new(sigma),
            impaired: floor.finish(),
        })
    }
}

/// The noise unit the CUSUM works in, and when it was not the sensor's own.
///
/// Quiet: the sensor noise. While the network rings: raised so the largest
/// recent swing counts as `ring_units` units, re-measured every quarter
/// settle window, and falling by at most half per window - ringing beats,
/// and one calm stretch does not mean it has died away.
struct Floor<'a> {
    series: Series<'a>,
    sigma: f64,
    band: f64,
    ring_units: f64,
    settle: usize,
    release: f64,
    /// The noise unit in use.
    scale: f64,
    /// Next sample at which to re-measure while ringing.
    next_check: usize,
    /// Start of the current blind or reduced-sensitivity stretch.
    since: Option<usize>,
    impaired: Vec<Span>,
}

impl<'a> Floor<'a> {
    fn new(series: Series<'a>, sigma: f64, settle: usize, config: &DetectorConfig) -> Self {
        Self {
            series,
            sigma,
            band: config.settle_band * sigma,
            ring_units: config.ring_units,
            settle,
            release: RELEASE_PER_WINDOW.powf(1.0 / 4.0),
            scale: sigma,
            next_check: usize::MAX,
            since: None,
            impaired: Vec::new(),
        }
    }

    /// Marks the sensor impaired from sample `k` (an event there).
    fn impaired_from(&mut self, k: usize) {
        self.since.get_or_insert(k);
    }

    /// Re-measures the floor on the settle window ending before sample
    /// `end`. Returns the window's median and whether it was quiet. With
    /// `may_fall` false the floor can only rise.
    fn measure(&mut self, end: usize, may_fall: bool) -> (f64, bool) {
        let w = &self.series.samples[end - self.settle..end];
        let centre = median(&mut w.to_vec());
        let spread = w.iter().map(|v| (v - centre).abs()).fold(0.0, f64::max);
        if spread <= self.band {
            self.scale = self.sigma;
            self.next_check = usize::MAX;
            if let Some(since) = self.since.take() {
                self.impaired.push(self.span(since, end));
            }
            return (centre, true);
        }
        let lowest = if may_fall {
            self.scale * self.release
        } else {
            self.scale
        };
        self.scale = self.sigma.max(spread / self.ring_units).max(lowest);
        self.next_check = end + (self.settle / 4).max(1);
        self.since.get_or_insert(end);
        (centre, false)
    }

    /// The impaired stretches, closing one still open at the record's end.
    fn finish(mut self) -> Vec<Span> {
        if let Some(since) = self.since.take() {
            let last = self.series.samples.len() - 1;
            self.impaired.push(self.span(since, last));
        }
        self.impaired
    }

    fn span(&self, from: usize, to: usize) -> Span {
        Span {
            from: self.series.time_at(to_f64(from)),
            to: self.series.time_at(to_f64(to)),
        }
    }
}

/// While ringing, the most the floor may fall per settle window.
const RELEASE_PER_WINDOW: f64 = 0.5;

#[allow(clippy::cast_precision_loss)] // sample indices are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
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

    #[test]
    fn listens_through_ringing_and_catches_a_second_event() {
        // A 4 m drop at t = 3 s, then the network rings (1.2 m, 1.7 s
        // period, dying away over 8 s), then a second 3 m drop at t = 20 s
        // while it still rings (about 0.15 m). 2 cm noise throughout.
        let mut rng = jk_kernel::random::Rng::new(7);
        let x: Vec<f64> = (0..128 * 30)
            .map(|k| {
                let t = f64::from(k) / 128.0;
                let mut h = 60.0 + 0.02 * rng.normal();
                if t >= 3.0 {
                    let age = t - 3.0;
                    h +=
                        -4.0 + 1.2 * (-age / 8.0).exp() * (std::f64::consts::TAU * age / 1.7).sin();
                }
                if t >= 20.0 {
                    h -= 3.0;
                }
                h
            })
            .collect();
        let config = DetectorConfig {
            settle_window: Time::new(2.0),
            ..DetectorConfig::default()
        };
        let d = Detector::new(config).unwrap().run(&series(&x)).unwrap();
        assert_eq!(d.events.len(), 2, "{:?}", d.events);
        assert!((d.events[0].arrival.value() - 3.0).abs() < 0.02);
        assert!((d.events[1].arrival.value() - 20.0).abs() < 0.05);
        // It came back while the network still rang, at reduced sensitivity.
        assert!(matches!(d.events[0].rearm, Rearm::Ringing { .. }));
        // ... and says so: impaired from the first event until at least the
        // second, so its silence in between is not claimed as "nothing".
        let first = d.impaired[0];
        assert!(
            first.from.value() <= 3.05 && first.to.value() >= 19.9,
            "{first:?}"
        );
    }
}
