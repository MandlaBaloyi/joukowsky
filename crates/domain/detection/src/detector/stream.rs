//! The live chain: samples arrive in pieces of any size.
//!
//! The same steps as a whole-record run, written as a state machine that
//! pauses whenever it needs samples that have not arrived yet: picking a
//! front needs a few samples after the alarm; re-arming needs the settle
//! window after the event. Whatever the piece sizes, the events and spans
//! are exactly those of the whole record (tested).
//!
//! One stream covers one CONTINUOUS stretch of samples. After a gap in the
//! data (telemetry context) a new stream starts and calibrates afresh: the
//! level may have moved while nobody watched.
//!
//! Memory is bounded: only the recent record that later steps can still
//! need is kept - at most `memory` (60 s by default) plus a settle window.

use jk_kernel::{Head, Time};

use super::floor::Floor;
use super::{DetectorConfig, Event, Impairment, Span, to_f64};
use crate::baseline::Baseline;
use crate::change::{Alarm, Cusum};
use crate::error::{DetectionError, positive};
use crate::noise::{median, sigma_from_differences};
use crate::onset;

/// What a piece of samples (or the end of the stream) brought.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct StreamOutput {
    /// Events whose front has been picked, in order of arrival.
    pub events: Vec<Event>,
    /// Stretches of impaired watching that have ended.
    pub impaired: Vec<Span>,
}

/// Where the chain is.
#[derive(Debug, Clone, Copy, PartialEq)]
enum State {
    /// Learning noise and level.
    Calibrating,
    /// Watching; the next sample to judge.
    Watching(usize),
    /// An alarm was raised; waiting for the samples to pick its front.
    Picking(Alarm),
    /// Deaf after an event; re-arms at this sample.
    Blind(usize),
}

/// What calibration learned, carried while watching.
#[derive(Debug, Clone, PartialEq)]
struct Watch {
    sigma: f64,
    baseline: Baseline,
    floor: Floor,
}

/// One sensor's detection chain over a continuous stretch of samples.
#[derive(Debug, Clone, PartialEq)]
pub struct DetectorStream {
    config: DetectorConfig,
    start: f64,
    dt: f64,
    calibration: usize,
    hold_off: usize,
    settle: usize,
    memory: usize,
    /// Kept samples; `buf[0]` is sample number `base`.
    buf: Vec<f64>,
    base: usize,
    state: State,
    watch: Option<Watch>,
    cusum: Cusum,
    /// An impaired stretch still going on: its first sample, and why.
    open: Option<(usize, Impairment)>,
}

impl DetectorStream {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // ceil of positive ratios
    pub(super) fn new(
        config: DetectorConfig,
        start: Time,
        interval: Time,
    ) -> Result<Self, DetectionError> {
        let dt = interval.value();
        positive("sampling interval", dt)?;
        let samples = |t: Time| (t.value() / dt).ceil() as usize;
        let calibration = samples(config.calibration).max(16);
        let settle = samples(config.settle_window).max(4);
        let reach = config.onset.margin + config.onset.before;
        Ok(Self {
            config,
            start: start.value(),
            dt,
            calibration,
            hold_off: samples(config.hold_off).max(1),
            settle,
            memory: samples(config.memory).max(settle + reach + 16),
            buf: Vec::new(),
            base: 0,
            state: State::Calibrating,
            watch: None,
            cusum: Cusum::new(config.cusum, calibration)?,
            open: Some((0, Impairment::Calibrating)),
        })
    }

    /// Samples needed before watching starts.
    #[must_use]
    pub fn calibration_samples(&self) -> usize {
        self.calibration
    }

    /// The sensor noise, once calibrated.
    #[must_use]
    pub fn noise(&self) -> Option<Head> {
        self.watch.as_ref().map(|w| Head::new(w.sigma))
    }

    /// Time of the last sample received (None before any).
    #[must_use]
    pub fn received_until(&self) -> Option<Time> {
        let len = self.len();
        (len > 0).then(|| self.time_at(to_f64(len - 1)))
    }

    /// An impaired stretch still going on: since when, and why.
    #[must_use]
    pub fn open_impairment(&self) -> Option<(Time, Impairment)> {
        self.open
            .map(|(from, why)| (self.time_at(to_f64(from)), why))
    }

    /// No event reported from now on can have arrived before this time:
    /// the sensor's past up to here is final. (While evidence of a change
    /// is building up, the watermark waits at where it began.)
    #[must_use]
    pub fn watermark(&self) -> Time {
        let earliest = match self.state {
            State::Calibrating => self.calibration,
            State::Watching(k) => self.cusum.pending_start().min(k),
            State::Picking(alarm) => alarm.change_start,
            State::Blind(at) => at,
        };
        self.time_at(to_f64(earliest) - to_f64(self.config.onset.margin + 1))
    }

    /// Feeds the next samples (continuing the stretch).
    ///
    /// # Errors
    /// A non-finite sample (gaps are the telemetry context's to report,
    /// never to fill - P6). A refused piece changes nothing.
    pub fn push(&mut self, samples: &[f64]) -> Result<StreamOutput, DetectionError> {
        if let Some(i) = samples.iter().position(|v| !v.is_finite()) {
            return Err(DetectionError::NonFiniteSample {
                index: self.len() + i,
            });
        }
        self.buf.extend_from_slice(samples);
        let mut out = StreamOutput::default();
        self.advance(&mut out);
        self.trim();
        Ok(out)
    }

    /// Ends the stretch: closes the impaired stretch still going on. A
    /// front still waiting for samples after it is not reported.
    #[must_use]
    pub fn finish(mut self) -> StreamOutput {
        let mut out = StreamOutput::default();
        let len = self.len();
        if len > 0 {
            self.close(len - 1, &mut out);
        }
        out
    }

    fn len(&self) -> usize {
        self.base + self.buf.len()
    }

    fn time_at(&self, k: f64) -> Time {
        Time::new(self.start + self.dt * k)
    }

    /// Runs the chain as far as the samples received allow.
    fn advance(&mut self, out: &mut StreamOutput) {
        loop {
            let len = self.len();
            match self.state {
                State::Calibrating => {
                    if len < self.calibration {
                        return;
                    }
                    let (quiet, scale) = self.calibrate();
                    self.close(self.calibration, out);
                    self.note_ringing(self.calibration, quiet, scale, out);
                    self.state = State::Watching(self.calibration);
                }
                State::Watching(k) => {
                    if k >= len {
                        return;
                    }
                    self.judge(k, out);
                }
                State::Picking(alarm) => {
                    let post_end = alarm.index + 1 + self.config.onset.after;
                    let swing_end = (alarm.index + 1).max(self.settle);
                    if len < post_end.max(swing_end) {
                        return;
                    }
                    self.pick(alarm, swing_end, out);
                }
                State::Blind(at) => {
                    if len <= at {
                        return;
                    }
                    self.rearm(at, out);
                }
            }
        }
    }

    /// Learns noise and level from the opening samples (base is still 0).
    /// Returns whether they were quiet, and the noise unit to start with.
    ///
    /// The noise comes from sample-to-sample differences, which ringing
    /// barely touches. But a stretch can begin while the network rings (a
    /// sensor back after a power cut, just after a burst): then watching
    /// starts at reduced sensitivity, exactly as after an event of its own
    /// - otherwise every echo would be taken for an event.
    fn calibrate(&mut self) -> (bool, f64) {
        let x = &self.buf[..self.calibration];
        let sigma = sigma_from_differences(x).max(self.config.min_noise.value());
        let alpha = (self.dt / self.config.baseline_time_constant.value()).min(1.0);
        // Warm the baseline up over the calibration window, so it has
        // already learned the drift trend when watching starts.
        let mut baseline = Baseline::new(median(&mut x[..x.len().min(16)].to_vec()), alpha);
        for &v in x {
            baseline.update(v);
        }
        let mut floor = Floor::new(sigma, self.settle, &self.config);
        let recent = &x[x.len() - self.settle.min(x.len())..];
        let (centre, quiet) = floor.measure(recent, self.calibration, true);
        if !quiet {
            // A trend learned from ringing means nothing: start level.
            baseline = Baseline::new(centre, alpha);
        }
        let scale = floor.scale;
        self.watch = Some(Watch {
            sigma,
            baseline,
            floor,
        });
        (quiet, scale)
    }

    /// Judges sample `k` while watching.
    fn judge(&mut self, k: usize, out: &mut StreamOutput) {
        let settle = self.settle;
        let base = self.base;
        let Some(w) = self.watch.as_mut() else { return };
        if k >= w.floor.next_check {
            let (centre, quiet) = w
                .floor
                .measure(&self.buf[k - settle - base..k - base], k, true);
            if !quiet {
                // Judge the next samples against the ringing's centre, the
                // same centre its size was measured around.
                let expected_now = w.baseline.forecast();
                w.baseline.shift(centre - expected_now);
            }
            let scale = w.floor.scale;
            self.note_ringing(k, quiet, scale, out);
        }
        let Some(w) = self.watch.as_mut() else { return };
        let v = self.buf[k - base];
        match self.cusum.push((v - w.baseline.forecast()) / w.floor.scale) {
            None => {
                w.baseline.update(v);
                self.state = State::Watching(k + 1);
            }
            Some(alarm) => self.state = State::Picking(alarm),
        }
    }

    /// Picks the front of `alarm`, or treats it as a swing of the ringing.
    fn pick(&mut self, alarm: Alarm, swing_end: usize, out: &mut StreamOutput) {
        let k = alarm.index;
        let reach = self.config.onset.margin + self.config.onset.before;
        let Some(w) = self.watch.as_ref() else { return };
        let (sigma, scale) = (w.sigma, w.floor.scale);
        // A change building up for longer than the memory is not a front.
        let recent = k - alarm.change_start.saturating_sub(reach) <= self.memory;
        let onset = match (recent, alarm.change_start.checked_sub(self.base)) {
            (true, Some(start)) => {
                onset::pick(&self.buf, start, k - self.base, &self.config.onset, scale)
            }
            _ => None,
        };
        let Some(o) = onset else {
            // A swing with no step behind it: ringing that crossed the
            // floor. Raise the floor to it and keep listening.
            let window_start = swing_end - self.settle;
            let (base, settle) = (self.base, self.settle);
            let Some(w) = self.watch.as_mut() else { return };
            let window = &self.buf[window_start - base..window_start - base + settle];
            let (_, quiet) = w.floor.measure(window, swing_end, false);
            let scale = w.floor.scale;
            w.baseline.update(self.buf[k - base]);
            self.cusum.reset_at(k + 1);
            self.note_ringing(swing_end, quiet, scale, out);
            self.state = State::Watching(k + 1);
            return;
        };
        out.events.push(Event {
            arrival: self.time_at(o.index + to_f64(self.base)),
            arrival_uncertainty: Time::new(self.dt * o.uncertainty),
            alarm: self.time_at(to_f64(k)),
            step: Head::new(o.post - o.pre),
            direction: alarm.direction,
            noise: Head::new(sigma),
        });
        self.close(k, out);
        self.open = Some((k, Impairment::Blind));
        // Blind for the hold-off and one settle window, then listen again.
        self.state = State::Blind(k + self.hold_off.max(self.settle));
    }

    /// Listens again at sample `at`, at the new level.
    fn rearm(&mut self, at: usize, out: &mut StreamOutput) {
        self.close(at, out);
        let (base, settle) = (self.base, self.settle);
        let Some(w) = self.watch.as_mut() else { return };
        let (centre, quiet) = w
            .floor
            .measure(&self.buf[at - settle - base..at - base], at, true);
        // Move the forecast to the new level; the trend is kept.
        let expected_now = w.baseline.forecast();
        w.baseline.shift(centre - expected_now);
        let scale = w.floor.scale;
        self.cusum.reset_at(at);
        self.note_ringing(at, quiet, scale, out);
        self.state = State::Watching(at);
    }

    /// Records the floor measured at sample `at`: a ringing stretch opens,
    /// grows or (when quiet) ends.
    fn note_ringing(&mut self, at: usize, quiet: bool, scale: f64, out: &mut StreamOutput) {
        match (quiet, self.open) {
            (true, Some((_, Impairment::Ringing { .. }))) => self.close(at, out),
            (false, None) => {
                self.open = Some((
                    at,
                    Impairment::Ringing {
                        floor: Head::new(scale),
                    },
                ));
            }
            (false, Some((from, Impairment::Ringing { floor }))) => {
                self.open = Some((
                    from,
                    Impairment::Ringing {
                        floor: Head::new(floor.value().max(scale)),
                    },
                ));
            }
            _ => {}
        }
    }

    /// Ends the impaired stretch going on, if any, at sample `at`.
    fn close(&mut self, at: usize, out: &mut StreamOutput) {
        if let Some((from, why)) = self.open.take() {
            out.impaired.push(Span {
                from: self.time_at(to_f64(from)),
                to: self.time_at(to_f64(at)),
                why,
            });
        }
    }

    /// Drops samples no later step can need.
    fn trim(&mut self) {
        let reach = self.config.onset.margin + self.config.onset.before;
        let keep_from = match self.state {
            State::Calibrating => 0,
            State::Watching(k) => k
                .saturating_sub(self.settle)
                .min(self.cusum.pending_start().saturating_sub(reach))
                .max(k.saturating_sub(self.memory)),
            State::Picking(alarm) => alarm
                .change_start
                .saturating_sub(reach)
                .min(((alarm.index + 1).max(self.settle)).saturating_sub(self.settle))
                .max(alarm.index.saturating_sub(self.memory)),
            State::Blind(at) => at.saturating_sub(self.settle.max(reach)),
        }
        .min(self.len());
        // Drain in large steps: moving the buffer costs as much as it holds.
        if keep_from > self.base && keep_from - self.base >= self.buf.len() / 2 {
            self.buf.drain(..keep_from - self.base);
            self.base = keep_from;
        }
    }
}
