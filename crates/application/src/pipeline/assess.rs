//! One event's alert: classify, locate, corroborate, grade.
//!
//! Given the detections fused into one event (at least one), this decides
//! what the engine says about it (`engine_overview.txt`, "Event pipeline"):
//!
//! - classify on the first sensor to see it (closest, clearest front);
//! - locate if two or more sensors saw it; a misfit above the limit marks
//!   the location untrusted;
//! - corroborate a one-sensor event: if other sensors watched at full
//!   sensitivity over the whole time a wave from anywhere would have
//!   reached them and felt nothing, it is not physical as seen (a sensor
//!   fault, or something too small or local to travel) - Review;
//! - name sensors whose data was not waited for (P6: silence is not "no
//!   events");
//! - grade: Alarm (confident, located burst), Warning (pump trip), Info
//!   (valve closure), Review (anything unsure or uncorroborated).

use jk_classification::{EventClass, Features, classify};
use jk_detection::Event;
use jk_kernel::Time;
use jk_localisation::{Arrival, localise};

use crate::alert::{Alert, Level};
use crate::pipeline::{PipelineConfig, Site};

/// A detection with everything later stages need.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Found {
    /// Index of the sensor.
    pub sensor: usize,
    /// As detected, on the sensor's clock.
    pub event: Event,
    /// Clock-corrected arrival, for fusion and localisation.
    pub arrival: Arrival,
    /// Classification features, once the record around the front is in
    /// (or the reason they could not be measured).
    pub features: Option<Result<Features, String>>,
}

/// A sensor whose data was not waited for: its index, and the time of its
/// last sample (None if it has sent nothing).
pub(super) type Unheard = (usize, Option<Time>);

/// The alert for the event seen by `members` (sorted by arrival).
/// `watched(sensor, from, to)` says whether a sensor watched at full
/// sensitivity over the whole of `[from, to]`.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)] // the grading, told in one place
pub(super) fn assess(
    site: &Site,
    config: &PipelineConfig,
    travel: &[Vec<f64>],
    members: &[Found],
    watched: &dyn Fn(usize, f64, f64) -> bool,
    unheard: &[Unheard],
    id: u32,
    revision: u32,
) -> Alert {
    let first = &members[0];
    let mut reasons = Vec::new();

    let (class, confidence) = match &first.features {
        Some(Ok(features)) => {
            let c = classify(
                features,
                &site.context,
                first.arrival.time,
                &config.classifier,
            );
            reasons.extend(c.reasons.iter().cloned());
            (c.class, c.confidence)
        }
        Some(Err(e)) => {
            reasons.push(format!("not classified: {e}"));
            (EventClass::Unknown, 0.0)
        }
        None => {
            reasons.push("not classified: the record after the front is not in yet".into());
            (EventClass::Unknown, 0.0)
        }
    };

    let arrivals: Vec<Arrival> = members.iter().map(|m| m.arrival).collect();
    let mut corroborated = true;
    let location = if arrivals.len() >= 2 {
        match localise(&site.network, &arrivals, &config.localise) {
            Ok(loc) => Some(loc),
            Err(e) => {
                reasons.push(format!("not located: {e}"));
                None
            }
        }
    } else {
        let me = first.sensor;
        let t = first.arrival.time.value();
        let watching: Vec<&str> = (0..site.sensors.len())
            .filter(|&j| j != me)
            .filter(|&j| {
                let tau = travel[me][j];
                tau.is_finite() && watched(j, t - tau, t + tau)
            })
            .map(|j| site.sensors[j].name.as_str())
            .collect();
        if watching.is_empty() {
            reasons.push(
                "seen by ONE sensor only, and no other sensor was watching: not located, \
                 not corroborated"
                    .into(),
            );
        } else {
            corroborated = false;
            reasons.push(format!(
                "seen by ONE sensor only: {} watched and felt nothing, though a pressure wave \
                 from anywhere in the network would have reached them - a fault in {}, or an \
                 event too small or too local to travel",
                watching.join(", "),
                site.sensors[me].name
            ));
        }
        None
    };
    let fits = location
        .as_ref()
        .is_some_and(|l| l.misfit <= config.max_misfit);
    if let Some(l) = &location
        && !fits
    {
        reasons.push(format!(
            "arrival times do not fit one source (misfit {:.1}): two events at once, a wrong \
             wave speed or a bad clock - location not trusted",
            l.misfit
        ));
    }
    if members.len() >= 2 {
        // Sensors that did not see it: say whose silence means nothing.
        let t = first.arrival.time.value();
        for (j, &tau) in travel[first.sensor].iter().enumerate() {
            let absent =
                members.iter().all(|m| m.sensor != j) && unheard.iter().all(|&(u, _)| u != j);
            if absent && tau.is_finite() && !watched(j, t - tau, t + tau) {
                reasons.push(format!(
                    "{} was not watching at full sensitivity when this wave passed (a gap in \
                     its data, calibrating, or ringing after an earlier event): its silence \
                     says nothing",
                    site.sensors[j].name
                ));
            }
        }
    }
    for &(j, last) in unheard {
        let name = &site.sensors[j].name;
        reasons.push(match last {
            Some(t) => format!(
                "{name}: no data after t = {:.1} s - not waited for longer; it could not confirm \
                 or help locate this event",
                t.value()
            ),
            None => {
                format!("{name}: no data at all - it could not confirm or help locate this event")
            }
        });
    }

    let level = match class {
        _ if !corroborated => Level::Review,
        EventClass::Burst if fits && confidence >= config.alarm_confidence => Level::Alarm,
        EventClass::PumpTrip => Level::Warning,
        EventClass::ValveClosure => Level::Info,
        EventClass::Burst | EventClass::Unknown => Level::Review,
    };
    let time = location.as_ref().map_or(first.arrival.time, |l| l.emission);
    Alert {
        id,
        revision,
        level,
        class,
        confidence,
        time: Time::new(time.value()),
        location,
        sensors: members
            .iter()
            .map(|m| site.sensors[m.sensor].name.clone())
            .collect(),
        reasons,
    }
}
