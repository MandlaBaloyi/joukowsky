//! The detection chain over one sensor's samples - live or all at once.
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
//!    - if its recent record is quiet (like plain noise), at full
//!      sensitivity;
//!    - if the network still rings, at REDUCED sensitivity (`floor`).
//!
//!    Waiting blind until the network is quiet was tried first and failed:
//!    a flowing network rang for over 25 s and a second burst went unseen.
//!    Every stretch of blind or reduced watching is reported (principle P2).
//!
//! The chain runs LIVE ([`DetectorStream`]: samples arrive in pieces of any
//! size) or over a whole record ([`Detector::run`]). The whole-record run
//! is the live one fed everything at once, so the two cannot disagree.
//!
//! - `floor`: the noise unit, raised while the network rings.
//! - `stream`: the live chain ([`DetectorStream`]).

mod floor;
mod stream;

pub use stream::{DetectorStream, StreamOutput};

use jk_kernel::{Head, Time};

pub use crate::change::Direction;
use crate::change::{Cusum, CusumConfig};
use crate::error::{DetectionError, positive};
use crate::onset::OnsetConfig;

/// The longest sampling interval the detector accepts [s]: Tier 1, one
/// sample a second, with 1% for a drifting logger clock. Slower data (Tier
/// 0, SCADA at 1-15 min) cannot show a wave, which crosses a site in
/// seconds; a pump switching between two readings would look like a front
/// (FN-046, found on L-Town's 5-minute data). Tiers:
/// `docs/03_sensing_and_signal/data_source_requirements.txt` section 1.
pub const SLOWEST_INTERVAL: f64 = 1.01;

/// One sensor's samples. Sample k is the average over the interval ending
/// at `start + k * interval` (how integrating loggers report).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Series<'a> {
    /// Pressure head samples (m).
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
    /// Floor on the noise estimate (m) (a perfectly quiet or heavily
    /// quantised signal must not make every tiny wiggle "infinitely"
    /// significant).
    pub min_noise: Head,
    /// Longest stretch of past record kept to pick a front from. A change
    /// that has been building up for longer (a very slow ramp) is not
    /// picked as a front.
    pub memory: Time,
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
            memory: Time::new(60.0),
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
    /// The sensor noise (learned at calibration) when it was seen.
    pub noise: Head,
}

/// Why a sensor was not watching at full sensitivity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Impairment {
    /// Learning the sensor's noise and level; not yet watching.
    Calibrating,
    /// Deaf after an event of its own, while the first echoes pass.
    Blind,
    /// Listening while the network rang, with the noise unit raised up to
    /// `floor`: events smaller than a few `floor`s could not be seen.
    Ringing {
        /// The highest noise unit used in the stretch.
        floor: Head,
    },
}

/// A stretch of time when a sensor was not watching at full sensitivity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Span {
    /// Start.
    pub from: Time,
    /// End.
    pub to: Time,
    /// Why.
    pub why: Impairment,
}

/// The configured detection chain.
#[derive(Debug, Clone, PartialEq)]
pub struct Detector {
    config: DetectorConfig,
}

/// What a whole-record run found.
#[derive(Debug, Clone, PartialEq)]
pub struct Detection {
    /// Events in order of arrival.
    pub events: Vec<Event>,
    /// The sensor noise learned during calibration.
    pub noise: Head,
    /// When the sensor was not watching at full sensitivity (calibrating,
    /// after its own events, while the network rang). Outside these spans
    /// its silence means "nothing happened"; inside them it does not
    /// (principles P2, P6).
    pub impaired: Vec<Span>,
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
        positive("memory", config.memory.value())?;
        Cusum::new(config.cusum, 0)?;
        Ok(Self { config })
    }

    /// The settings.
    #[must_use]
    pub fn config(&self) -> &DetectorConfig {
        &self.config
    }

    /// Starts watching a continuous stretch of samples, the first taken at
    /// `start`, one every `interval`. Feed it with [`DetectorStream::push`].
    ///
    /// # Errors
    /// A non-positive interval.
    pub fn stream(&self, start: Time, interval: Time) -> Result<DetectorStream, DetectionError> {
        DetectorStream::new(self.config, start, interval)
    }

    /// Runs the chain over a whole record.
    ///
    /// # Errors
    /// A non-positive interval, a non-finite sample, or a series too short
    /// to calibrate on.
    pub fn run(&self, series: &Series<'_>) -> Result<Detection, DetectionError> {
        let mut stream = self.stream(series.start, series.interval)?;
        let needed = stream.calibration_samples() + self.config.onset.after + 2;
        if series.samples.len() < needed {
            return Err(DetectionError::TooShort {
                samples: series.samples.len(),
                needed,
            });
        }
        let mut found = stream.push(series.samples)?;
        let noise = stream.noise().unwrap_or(self.config.min_noise);
        let rest = stream.finish();
        found.events.extend(rest.events);
        found.impaired.extend(rest.impaired);
        Ok(Detection {
            events: found.events,
            noise,
            impaired: found.impaired,
        })
    }
}

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
    fn a_short_surge_pulse_is_picked_as_a_front() {
        // A surge with no lasting step (G-23): 4 s quiet at 30 m, then a
        // 5 m pulse lasting 0.1 s (13 samples - the wave's round trip on a
        // 50 m pipe), then back to 30 m. The front picker compares the
        // level before with the 6 samples after the alarm, so a pulse
        // longer than those (47 ms) is a front, not "ringing".
        let x: Vec<f64> = (0..768)
            .map(|k| if (512..525).contains(&k) { 35.0 } else { 30.0 })
            .collect();
        let d = Detector::new(DetectorConfig::default())
            .unwrap()
            .run(&series(&x))
            .unwrap();
        assert_eq!(d.events.len(), 1, "{:?}", d.events);
        let e = d.events[0];
        assert_eq!(e.direction, Direction::Rise);
        assert!((e.step.value() - 5.0).abs() < 1e-9, "{}", e.step);
        assert!((e.arrival.value() - 511.0 / 128.0).abs() < 1.0 / 128.0);
    }

    #[test]
    fn refuses_data_too_slow_to_see_a_wave() {
        // SCADA every 5 minutes (L-Town, BattLeDIM 2020): no fronts looked
        // for (P2). One a second (Tier 1) is still watched.
        let det = Detector::new(DetectorConfig::default()).unwrap();
        let x = vec![3.5; 600];
        let slow = Series {
            samples: &x,
            start: Time::ZERO,
            interval: Time::new(300.0),
        };
        assert!(matches!(
            det.run(&slow),
            Err(DetectionError::TooSlow { interval, .. }) if (interval - 300.0).abs() < 1e-12
        ));
        assert!(det.stream(Time::ZERO, Time::new(1.0)).is_ok());
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

    /// A 4 m drop at t = 3 s, then the network rings (1.2 m, 1.7 s period,
    /// dying away over 8 s), then a second 3 m drop at t = 20 s while it
    /// still rings (about 0.15 m). 2 cm noise throughout.
    fn ringing_record() -> Vec<f64> {
        let mut rng = jk_kernel::random::Rng::new(7);
        (0..128 * 30)
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
            .collect()
    }

    fn ringing_config() -> DetectorConfig {
        DetectorConfig {
            settle_window: Time::new(2.0),
            ..DetectorConfig::default()
        }
    }

    #[test]
    fn listens_through_ringing_and_catches_a_second_event() {
        let x = ringing_record();
        let d = Detector::new(ringing_config())
            .unwrap()
            .run(&series(&x))
            .unwrap();
        assert_eq!(d.events.len(), 2, "{:?}", d.events);
        assert!((d.events[0].arrival.value() - 3.0).abs() < 0.02);
        assert!((d.events[1].arrival.value() - 20.0).abs() < 0.05);
        // It says when it was not fully watching: calibrating, deaf after
        // the first event, then listening through the ringing at reduced
        // sensitivity until at least the second event - so its silence in
        // between is not claimed as "nothing happened".
        let kinds: Vec<_> = d.impaired.iter().map(|s| s.why).collect();
        assert!(matches!(kinds[0], Impairment::Calibrating));
        assert!(matches!(kinds[1], Impairment::Blind));
        assert!(matches!(kinds[2], Impairment::Ringing { .. }));
        assert!(d.impaired[1].from.value() <= 3.05);
        assert!(d.impaired[2].to.value() >= 19.9, "{:?}", d.impaired[2]);
    }

    #[test]
    fn live_and_whole_record_runs_agree_for_any_chunk_sizes() {
        // The same record fed in pieces of 1, 7, 128 and random sizes
        // gives exactly the same events and spans as all at once.
        let x = ringing_record();
        let det = Detector::new(ringing_config()).unwrap();
        let whole = det.run(&series(&x)).unwrap();
        let mut rng = jk_kernel::random::Rng::new(11);
        for size in [1, 7, 128, 0] {
            let mut s = det.stream(Time::ZERO, Time::new(1.0 / 128.0)).unwrap();
            let mut events = Vec::new();
            let mut spans = Vec::new();
            let mut i = 0;
            while i < x.len() {
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                let n = if size == 0 {
                    1 + (rng.uniform() * 300.0) as usize
                } else {
                    size
                };
                let out = s.push(&x[i..(i + n).min(x.len())]).unwrap();
                events.extend(out.events);
                spans.extend(out.impaired);
                i += n;
            }
            let out = s.finish();
            events.extend(out.events);
            spans.extend(out.impaired);
            assert_eq!(events, whole.events, "chunk size {size}");
            assert_eq!(spans, whole.impaired, "chunk size {size}");
        }
    }
}
