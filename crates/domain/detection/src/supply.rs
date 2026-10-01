//! Supply watch: when a sensor's pipe stood EMPTY, and when the supply came
//! back (`docs/04_detection_and_localisation/event_detection.txt`, "Supply
//! states"; `docs/02_physics_of_transients/refilling_and_air_pockets.txt`).
//!
//! The front detector ([`crate::detector`]) looks for waves - changes over
//! milliseconds to a second - and deliberately lets slow ramps pass as
//! drift. A pipe refilling against trapped air is such a ramp: the air
//! ahead of the returning water is squeezed over a minute or more (FN-033).
//! What marks it is the state BEFORE: a drained pipe reads about 0 (air at
//! atmospheric pressure), or below in a vacuum, for minutes to hours. This
//! watch finds those spells and when each ended - the moment the supply
//! returned and refilling began.
//!
//! A spell must last [`SupplyConfig::min_empty`]: a pump trip's down-surge
//! can touch the vapour floor for seconds, which is a transient, not an
//! empty pipe. ANY reading above the threshold ends a spell: noise (a few
//! cm) never reaches it, so water has arrived - and after a slam the
//! pressure swings between the surge and the floor every 2L/a, so waiting
//! for it to stay up would miss the return (FN-033). A flicker of supply
//! that drains again is a refill attempt, and counts. A spell running into the start or end of the record says so (`None`): when it
//! began, or whether it has ended, is not known (principle P6).

use jk_kernel::{Head, Time};

use crate::DetectionError;
use crate::detector::Series;

/// What counts as an empty pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupplyConfig {
    /// At or below this pressure head the pipe at the sensor is empty or
    /// unpressurised.
    pub empty_below: Head,
    /// Shortest spell counted as an empty pipe (shorter ones are
    /// transients).
    pub min_empty: Time,
}

impl Default for SupplyConfig {
    fn default() -> Self {
        Self {
            empty_below: Head::new(2.0),
            min_empty: Time::new(60.0),
        }
    }
}

/// A stretch during which the pipe at the sensor stood empty.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EmptySpell {
    /// When it emptied - `None` if it was already empty when the record
    /// began.
    pub from: Option<Time>,
    /// When the supply returned (the first sample above `empty_below`) -
    /// `None` if it was still empty when the record ended.
    pub until: Option<Time>,
}

/// The supply watch, live: one continuous stretch of a sensor's record,
/// fed in pieces of any size - the same answers as [`empty_spells`] over
/// the whole stretch.
#[derive(Debug, Clone, PartialEq)]
pub struct SupplyWatch {
    config: SupplyConfig,
    start: f64,
    interval: f64,
    /// Samples taken in so far.
    seen: usize,
    /// The first sample of the current spell.
    current: Option<usize>,
}

impl SupplyWatch {
    /// A watch over a stretch starting at `start`, one sample every
    /// `interval`.
    ///
    /// # Errors
    /// A setting out of range.
    pub fn new(config: SupplyConfig, start: Time, interval: Time) -> Result<Self, DetectionError> {
        for (name, value) in [
            ("interval", interval.value()),
            ("min_empty", config.min_empty.value()),
        ] {
            if !(value > 0.0 && value.is_finite()) {
                return Err(DetectionError::InvalidParameter {
                    name,
                    value,
                    requirement: "positive and finite",
                });
            }
        }
        Ok(Self {
            config,
            start: start.value(),
            interval: interval.value(),
            seen: 0,
            current: None,
        })
    }

    /// Takes the next `samples`; returns the spells that ended in them
    /// (the supply came back).
    ///
    /// # Errors
    /// A non-finite sample (its index within `samples`); nothing is taken.
    pub fn push(&mut self, samples: &[f64]) -> Result<Vec<EmptySpell>, DetectionError> {
        if let Some(index) = samples.iter().position(|x| !x.is_finite()) {
            return Err(DetectionError::NonFiniteSample { index });
        }
        let empty = self.config.empty_below.value();
        let mut ended = Vec::new();
        for &x in samples {
            let k = self.seen;
            self.seen += 1;
            if x <= empty {
                self.current.get_or_insert(k);
            } else if let Some(first) = self.current.take() {
                ended.extend(self.spell(first, k, true));
            }
        }
        Ok(ended)
    }

    /// The stretch has ended (end of record, or a gap): the spell still
    /// open, if long enough - not known to have ended.
    #[must_use]
    pub fn finish(self) -> Option<EmptySpell> {
        self.current
            .and_then(|first| self.spell(first, self.seen, false))
    }

    #[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
    fn span(&self, a: usize, b: usize) -> f64 {
        (b - a) as f64 * self.interval
    }

    #[allow(clippy::cast_precision_loss)]
    fn at(&self, k: usize) -> Time {
        Time::new(self.start + k as f64 * self.interval)
    }

    fn spell(&self, first: usize, end: usize, ended: bool) -> Option<EmptySpell> {
        (self.span(first, end) >= self.config.min_empty.value()).then(|| EmptySpell {
            from: (first > 0).then(|| self.at(first)),
            until: ended.then(|| self.at(end)),
        })
    }
}

/// The empty spells in `series`, in order.
///
/// # Errors
/// A setting out of range, or a non-finite sample.
pub fn empty_spells(
    series: &Series<'_>,
    config: &SupplyConfig,
) -> Result<Vec<EmptySpell>, DetectionError> {
    let mut watch = SupplyWatch::new(*config, series.start, series.interval)?;
    let mut spells = watch.push(series.samples)?;
    spells.extend(watch.finish());
    Ok(spells)
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 10.0;

    fn spells(x: &[f64]) -> Vec<EmptySpell> {
        let series = Series {
            samples: x,
            start: Time::ZERO,
            interval: Time::new(1.0 / RATE),
        };
        empty_spells(&series, &SupplyConfig::default()).unwrap()
    }

    /// `secs` seconds of `value`.
    fn hold(value: f64, secs: f64) -> Vec<f64> {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n = (secs * RATE).round() as usize;
        vec![value; n]
    }

    #[test]
    fn a_cut_and_a_return_are_found() {
        let x = [hold(30.0, 10.0), hold(0.0, 100.0), hold(30.0, 10.0)].concat();
        let s = spells(&x);
        assert_eq!(s.len(), 1);
        assert!((s[0].from.unwrap().value() - 10.0).abs() < 1e-9);
        assert!((s[0].until.unwrap().value() - 110.0).abs() < 1e-9);
    }

    #[test]
    fn a_brief_touch_of_zero_is_a_transient_not_an_empty_pipe() {
        let x = [hold(30.0, 10.0), hold(-9.0, 5.0), hold(30.0, 10.0)].concat();
        assert!(spells(&x).is_empty());
    }

    #[test]
    fn a_flicker_of_supply_is_a_return() {
        let x = [
            hold(0.0, 70.0),
            hold(2.5, 1.0),
            hold(0.0, 70.0),
            hold(30.0, 5.0),
        ]
        .concat();
        let s = spells(&x);
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].from, None, "empty since before the record");
        assert!((s[0].until.unwrap().value() - 70.0).abs() < 1e-9);
        assert!((s[1].from.unwrap().value() - 71.0).abs() < 1e-9);
        assert!((s[1].until.unwrap().value() - 141.0).abs() < 1e-9);
    }

    #[test]
    fn a_slam_swinging_back_to_the_floor_still_marks_the_return() {
        // Water hammer after a slam: up for 1 s, down for 1 s, ...
        let mut x = hold(0.0, 100.0);
        for _ in 0..5 {
            x.extend(hold(379.0, 1.0));
            x.extend(hold(-10.0, 1.0));
        }
        let s = spells(&x);
        assert_eq!(s.len(), 1);
        assert!((s[0].until.unwrap().value() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn live_in_pieces_gives_the_same_spells() {
        let x = [hold(30.0, 10.0), hold(0.0, 100.0), hold(30.0, 10.0)].concat();
        let mut watch =
            SupplyWatch::new(SupplyConfig::default(), Time::ZERO, Time::new(1.0 / RATE)).unwrap();
        let mut live = Vec::new();
        for piece in x.chunks(7) {
            live.extend(watch.push(piece).unwrap());
        }
        live.extend(watch.finish());
        assert_eq!(live, spells(&x));
    }

    #[test]
    fn still_empty_at_the_end_is_said() {
        let x = [hold(30.0, 5.0), hold(0.0, 70.0)].concat();
        let s = spells(&x);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].until, None);
    }
}
