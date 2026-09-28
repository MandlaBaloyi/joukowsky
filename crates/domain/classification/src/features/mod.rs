//! Physical features of an event, measured from one sensor's record
//! (`event_classification.txt` section 2).
//!
//! - [`front`]: the initial front - direction, how far it swung, how SHARP
//!   it was (10-90% rise time). A burst opens in milliseconds; a pump runs
//!   down over a second.
//! - [`after`]: what the signal did next - did it stay at the new level
//!   (persistence)?
//!
//! Features are physics, not black-box statistics: each has a physical
//! meaning an engineer can check (principle P1). Later: dominant period and
//! damping (Prony / matrix pencil), time at the vapour floor, arrival order
//! across sensors.

pub mod after;
pub mod front;

use jk_detection::{Direction, Event, Series};
use jk_kernel::{Head, Time};

use crate::error::{ClassificationError, positive};

/// How much record the features look at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureConfig {
    /// Record before the arrival used for the resting level.
    pub before: Time,
    /// Record after the arrival examined for the front's swing.
    pub front_window: Time,
    /// Record after the arrival examined for what happened next.
    pub after_window: Time,
}

impl Default for FeatureConfig {
    fn default() -> Self {
        Self {
            before: Time::new(0.5),
            front_window: Time::new(1.0),
            after_window: Time::new(3.0),
        }
    }
}

/// The measured features of one event at one sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Features {
    /// Which way the front moved.
    pub direction: Direction,
    /// Level just before the front.
    pub level_before: Head,
    /// Largest excursion from that level within the front window, signed.
    pub swing: Head,
    /// 10-90% rise time of that swing.
    pub rise_time: Time,
    /// Size of the swing in units of the sensor noise.
    pub swing_in_noise: f64,
    /// Mean deviation over the after-window as a fraction of the swing:
    /// ~1 = stayed at the new level, ~0 = came back.
    pub persistence: f64,
}

/// Measures the features of `event` in `series`, a sensor with noise
/// standard deviation `noise`.
///
/// # Errors
/// Bad windows, or too little record around the event.
pub fn extract(
    series: &Series<'_>,
    event: &Event,
    noise: Head,
    config: &FeatureConfig,
) -> Result<Features, ClassificationError> {
    positive("before", config.before.value())?;
    positive("front_window", config.front_window.value())?;
    positive("after_window", config.after_window.value())?;
    positive("noise", noise.value())?;
    let dt = series.interval.value();
    positive("sampling interval", dt)?;
    let index_of = |t: f64| (t - series.start.value()) / dt;
    let arrival = index_of(event.arrival.value());
    let need = || ClassificationError::NotEnoughRecord {
        before: config.before.value(),
        after: config.after_window.value().max(config.front_window.value()),
    };
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // checked >= 0
    let at = |i: f64| -> Option<usize> { (i >= 0.0).then(|| i.floor() as usize) };
    let pre_start = at(arrival - config.before.value() / dt).ok_or_else(need)?;
    let pre_end = at(arrival - 1.0).ok_or_else(need)?;
    let front_end = at(arrival + config.front_window.value() / dt).ok_or_else(need)?;
    let after_end = at(arrival + config.after_window.value() / dt).ok_or_else(need)?;
    if after_end >= series.samples.len()
        || front_end >= series.samples.len()
        || pre_end <= pre_start
    {
        return Err(need());
    }

    let x = series.samples;
    let level_before = jk_detection::noise::median(&mut x[pre_start..pre_end].to_vec());
    let sign = match event.direction {
        Direction::Rise => 1.0,
        Direction::Drop => -1.0,
    };
    let from = pre_end;
    let swing = front::swing(&x[from..=front_end], level_before, sign);
    let rise_samples = front::rise_samples(&x[from..=front_end], level_before, swing);
    let persistence = after::persistence(&x[from..=after_end], level_before, swing);
    Ok(Features {
        direction: event.direction,
        level_before: Head::new(level_before),
        swing: Head::new(swing),
        rise_time: Time::new(rise_samples * dt),
        swing_in_noise: swing.abs() / noise.value(),
        persistence,
    })
}
