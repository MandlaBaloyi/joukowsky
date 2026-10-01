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
//! empty pipe. Readings that wander above the threshold for less than
//! [`SupplyConfig::bridge`] (noise, a gurgle) do not end a spell. A spell
//! running into the start or end of the record says so (`None`): when it
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
    /// Readings above `empty_below` for less than this do not end a spell.
    pub bridge: Time,
}

impl Default for SupplyConfig {
    fn default() -> Self {
        Self {
            empty_below: Head::new(2.0),
            min_empty: Time::new(60.0),
            bridge: Time::new(2.0),
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

/// The empty spells in `series`, in order.
///
/// # Errors
/// A setting out of range, or a non-finite sample.
pub fn empty_spells(
    series: &Series<'_>,
    config: &SupplyConfig,
) -> Result<Vec<EmptySpell>, DetectionError> {
    let dt = series.interval.value();
    for (name, value) in [
        ("interval", dt),
        ("min_empty", config.min_empty.value()),
        ("bridge", config.bridge.value()),
    ] {
        if !(value > 0.0 && value.is_finite()) {
            return Err(DetectionError::InvalidParameter {
                name,
                value,
                requirement: "positive and finite",
            });
        }
    }
    if let Some(index) = series.samples.iter().position(|x| !x.is_finite()) {
        return Err(DetectionError::NonFiniteSample { index });
    }
    let empty = config.empty_below.value();
    #[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
    let span = |a: usize, b: usize| (b - a) as f64 * dt;
    #[allow(clippy::cast_precision_loss)]
    let at = |k: usize| series.time_at(k as f64);
    let mut spells = Vec::new();
    let mut keep = |first: usize, end: usize, ended: bool| {
        if span(first, end) >= config.min_empty.value() {
            spells.push(EmptySpell {
                from: (first > 0).then(|| at(first)),
                until: ended.then(|| at(end)),
            });
        }
    };
    // The current spell: its first empty sample and its last one so far.
    let mut current: Option<(usize, usize)> = None;
    for (k, &x) in series.samples.iter().enumerate() {
        match current {
            _ if x <= empty => current = Some((current.map_or(k, |(first, _)| first), k)),
            Some((first, last)) if span(last + 1, k + 1) >= config.bridge.value() => {
                keep(first, last + 1, true);
                current = None;
            }
            _ => {}
        }
    }
    if let Some((first, last)) = current {
        keep(first, last + 1, false);
    }
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
    fn a_short_wander_above_does_not_end_the_spell() {
        let x = [
            hold(0.0, 50.0),
            hold(2.5, 1.0),
            hold(0.0, 50.0),
            hold(30.0, 5.0),
        ]
        .concat();
        let s = spells(&x);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].from, None, "empty since before the record");
        assert!((s[0].until.unwrap().value() - 101.0).abs() < 1e-9);
    }

    #[test]
    fn still_empty_at_the_end_is_said() {
        let x = [hold(30.0, 5.0), hold(0.0, 70.0)].concat();
        let s = spells(&x);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].until, None);
    }
}
