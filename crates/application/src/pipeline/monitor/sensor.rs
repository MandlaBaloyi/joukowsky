//! What the monitor keeps per sensor.

use jk_detection::{DetectorStream, Series};
use jk_kernel::Time;
use jk_telemetry::Assembler;

/// One sensor's live state.
#[derive(Debug, Clone, Default)]
pub(super) struct SensorState {
    /// Continuity and gaps of its data (from its first chunk on).
    pub assembler: Option<Assembler>,
    /// Detection over the current continuous stretch.
    pub stream: Option<DetectorStream>,
    /// Closed stretches when it was not watching at full sensitivity:
    /// calibrating, blind, ringing, and gaps in its data [s].
    pub impaired: Vec<(f64, f64)>,
    /// Time of its first valid sample.
    pub first_data: Option<f64>,
    /// Recent valid samples of the current stretch (for classification):
    /// sample i is sample `dropped + i` of the stretch.
    history: Vec<f64>,
    stretch_start: f64,
    dropped: usize,
}

impl SensorState {
    /// Its sampling interval, once data has arrived.
    pub fn interval(&self) -> Option<f64> {
        self.assembler.as_ref().map(|a| a.interval().value())
    }

    /// Time of the last sample received.
    pub fn received_until(&self) -> Option<f64> {
        self.assembler
            .as_ref()
            .and_then(Assembler::received_until)
            .map(Time::value)
    }

    /// No detection reported from now on can have arrived before this.
    pub fn final_until(&self) -> f64 {
        match (&self.stream, self.received_until(), self.interval()) {
            (Some(s), _, _) => s.watermark().value(),
            (None, Some(t), Some(dt)) => t + dt,
            _ => f64::NEG_INFINITY,
        }
    }

    /// Whether it watched at full sensitivity over the whole of
    /// `[from, to]`: data covers it, and nothing impaired it.
    pub fn watched(&self, from: f64, to: f64) -> bool {
        let (Some(first), Some(until)) = (self.first_data, self.received_until()) else {
            return false;
        };
        let open_since = [
            self.stream
                .as_ref()
                .and_then(DetectorStream::open_impairment)
                .map(|(t, _)| t.value()),
            self.assembler
                .as_ref()
                .and_then(Assembler::open_gap)
                .map(|(t, _)| t.value()),
        ];
        first <= from
            && until >= to
            && self.impaired.iter().all(|&(a, b)| b < from || a > to)
            && open_since.iter().flatten().all(|&a| a > to)
    }

    /// Every stretch when it was not watching at full sensitivity, the
    /// open one (if any) running to its last sample [s].
    pub fn impaired_spans(&self) -> Vec<(f64, f64)> {
        let mut spans = self.impaired.clone();
        let until = self.received_until().unwrap_or(f64::NEG_INFINITY);
        let open = [
            self.stream
                .as_ref()
                .and_then(DetectorStream::open_impairment)
                .map(|(t, _)| t.value()),
            self.assembler
                .as_ref()
                .and_then(Assembler::open_gap)
                .map(|(t, _)| t.value()),
        ];
        spans.extend(open.iter().flatten().map(|&from| (from, until)));
        spans
    }

    /// A new continuous stretch begins at `start`.
    pub fn start_stretch(&mut self, start: f64) {
        self.history.clear();
        self.stretch_start = start;
        self.dropped = 0;
        self.first_data.get_or_insert(start);
    }

    /// Valid samples continuing the stretch.
    pub fn append(&mut self, samples: &[f64]) {
        self.history.extend_from_slice(samples);
    }

    /// The recent record of the stretch, sampled every `interval`.
    pub fn history(&self, interval: f64) -> Series<'_> {
        Series {
            samples: &self.history,
            start: Time::new(self.stretch_start + interval * to_f64(self.dropped)),
            interval: Time::new(interval),
        }
    }

    /// Drops recent-record samples before time `t` (in large steps).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // checked >= 0
    pub fn forget_before(&mut self, t: f64, interval: f64) {
        let first = (t - self.stretch_start) / interval;
        if !(first.is_finite() && first > 0.0) {
            return;
        }
        let n = (first.floor() as usize)
            .saturating_sub(self.dropped)
            .min(self.history.len());
        if n > 0 && n >= self.history.len() / 2 {
            self.history.drain(..n);
            self.dropped += n;
        }
    }

    /// Forgets impaired stretches that ended before `t`.
    pub fn forget_impaired_before(&mut self, t: f64) {
        self.impaired.retain(|&(_, to)| to >= t);
    }
}

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}
