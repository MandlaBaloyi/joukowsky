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
use jk_detection::{Direction, Event};
use jk_kernel::clock::stamp;
use jk_kernel::{Head, Time};
use jk_localisation::{Arrival, Devices, Drop, localise_with, size_with};

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
    /// When supply last returned to this sensor's empty pipe before the
    /// event (clock-corrected), if the supply watch saw it.
    pub supply_return: Option<Time>,
    /// For an event that took the pressure to or below atmospheric: the
    /// highest reading in the follow-up after it, and over how long a
    /// record (measured later; the alert is then revised).
    pub peak_after: Option<(Head, Time)>,
}

impl Found {
    /// Whether this detection took the pressure to or below atmospheric,
    /// so what follows (a collapse) is worth watching for.
    pub fn wants_follow_up(&self) -> bool {
        matches!(&self.features, Some(Ok(f)) if f.low.lowest.value() <= 0.0)
    }
}

/// What each member sensor saw below zero: negative pressure (water can be
/// drawn in) and the vapour floor (the column separated).
fn low_pressure_reasons(site: &Site, config: &PipelineConfig, members: &[Found]) -> Vec<String> {
    // A reading this close to an end of the logger's range may be clipped.
    const CLIP: f64 = 0.2;
    let mut reasons = Vec::new();
    for m in members {
        let Some(Ok(f)) = &m.features else { continue };
        let sensor = &site.sensors[m.sensor];
        let name = &sensor.name;
        let window = config.features.after_window.value();
        let floor = config.features.vapour_floor.value();
        if let Some((lowest, _)) = sensor.range
            && f.low.lowest.value() <= lowest.value() + CLIP
            && lowest.value() > floor + config.features.floor_band.value()
        {
            reasons.push(format!(
                "{name}: the reading held at the logger's minimum ({:.1} m) - the real pressure \
                 may have gone lower, as far as the vapour floor ({floor:.1} m): negative \
                 pressure and column separation cannot be ruled out with this logger (P2)",
                lowest.value()
            ));
        }
        if f.low.at_floor.value() > 0.0 {
            reasons.push(format!(
                "{name}: COLUMN SEPARATION - the pressure sat at the vapour floor ({:.1} m) for \
                 {:.2} s of the {window:.0} s after the front: the water boiled and the column \
                 split. When it rejoins, the collapse can reach about twice the usual surge \
                 (FN-035) - check the pipe's pressure rating",
                config.features.vapour_floor.value(),
                f.low.at_floor.value()
            ));
        }
        if f.low.below_zero.value() > 0.0 {
            reasons.push(format!(
                "{name}: NEGATIVE pressure for {:.2} s, down to {:.1} m: water outside the pipe \
                 can be drawn in through leaks and joints - a water-quality risk (intrusion)",
                f.low.below_zero.value(),
                f.low.lowest.value()
            ));
        }
        if let Some((peak, span)) = m.peak_after {
            let before = f.level_before.value();
            let clipped = sensor
                .range
                .is_some_and(|(_, highest)| peak.value() >= highest.value() - CLIP);
            reasons.push(format!(
                "{name}: in the {:.0} s after the front the pressure peaked at {:.1} m - {:.1} \
                 times the {before:.1} m before it{}",
                span.value(),
                peak.value(),
                peak.value() / before,
                if clipped {
                    " (at the logger's maximum: the real peak may have been higher)"
                } else {
                    ""
                }
            ));
        }
    }
    reasons
}

/// What each member sensor saw against the pressure rating of the pipes
/// at its node (the lowest of them): the highest pressure after the front,
/// or the collapse peak if measured since.
fn rating_reasons(site: &Site, members: &[Found]) -> Vec<String> {
    let mut reasons = Vec::new();
    for m in members {
        let Some(Ok(f)) = &m.features else { continue };
        let sensor = &site.sensors[m.sensor];
        let rating = site
            .network
            .pipes()
            .iter()
            .filter(|p| p.from == sensor.node || p.to == sensor.node)
            .filter_map(|p| p.pipe.rating())
            .map(Head::value)
            .fold(f64::INFINITY, f64::min);
        if !rating.is_finite() {
            continue;
        }
        let highest = m.peak_after.map_or(f.low.highest.value(), |(p, _)| {
            p.value().max(f.low.highest.value())
        });
        let before = f.level_before.value();
        if before > rating {
            reasons.push(format!(
                "{}: OVER RATING IN NORMAL OPERATION - the pressure already stood at {before:.1} m \
                 before the event, above the {rating:.1} m the pipes there are rated for; the \
                 event took it to {highest:.1} m",
                sensor.name
            ));
        } else if highest > rating {
            reasons.push(format!(
                "{}: OVER RATING - the surge took the pressure to {highest:.1} m, {:.2} times the \
                 {rating:.1} m the pipes there are rated for: inspect them for damage; each such \
                 event also spends their fatigue life",
                sensor.name,
                highest / rating
            ));
        }
    }
    reasons
}

/// A sensor whose data was not waited for: its index, and the time of its
/// last sample (None if it has sent nothing).
pub(super) type Unheard = (usize, Option<Time>);

/// The alert for the event seen by `members` (sorted by arrival), with the
/// pumps and valves as `devices` has them (and words saying so), and
/// `travel` the sensor-to-sensor wave times under them.
/// `watched(sensor, from, to)` says whether a sensor watched at full
/// sensitivity over the whole of `[from, to]`.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)] // the grading, told in one place
pub(super) fn assess(
    site: &Site,
    config: &PipelineConfig,
    (devices, hour): (&Devices, Option<String>),
    travel: &[Vec<f64>],
    members: &[Found],
    watched: &dyn Fn(usize, f64, f64) -> bool,
    unheard: &[Unheard],
    id: u32,
    revision: u32,
) -> Alert {
    let first = &members[0];
    let mut reasons = Vec::new();
    reasons.extend(hour);

    let (class, confidence) = match &first.features {
        Some(Ok(features)) => {
            let context = site
                .context
                .clone()
                .with_supply_returns(first.supply_return.as_slice());
            let c = classify(features, &context, first.arrival.time, &config.classifier);
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
        match localise_with(&site.network, devices, &arrivals, &config.localise) {
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
                "seen by ONE sensor only, and no other sensor that its wave could reach was \
                 watching: not located, not corroborated"
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
                "{name}: no data after {} - not waited for longer; it could not confirm or help \
                 locate this event",
                stamp(t.value(), site.utc_offset())
            ),
            None => {
                format!("{name}: no data at all - it could not confirm or help locate this event")
            }
        });
    }

    let level = match class {
        _ if !corroborated => Level::Review,
        EventClass::Burst if fits && confidence >= config.alarm_confidence => Level::Alarm,
        EventClass::PumpTrip | EventClass::Refill | EventClass::SupplyCut => Level::Warning,
        EventClass::ValveClosure => Level::Info,
        EventClass::Burst | EventClass::Unknown => Level::Review,
    };
    // Below zero, or above the pipes' rating: whatever the class, dangers
    // of their own.
    let mut low = low_pressure_reasons(site, config, members);
    low.extend(rating_reasons(site, members));
    let level = if low.is_empty() || !corroborated {
        level
    } else {
        level.max(Level::Warning)
    };
    reasons.extend(low);
    let time = location.as_ref().map_or(first.arrival.time, |l| l.emission);
    // How big: for a located burst, from the drops the loggers measured.
    let size = match (&location, class) {
        (Some(l), EventClass::Burst) if fits => {
            let drops: Vec<Drop> = members
                .iter()
                .filter(|m| m.event.direction == Direction::Drop)
                .map(|m| Drop {
                    node: site.sensors[m.sensor].node,
                    drop: Head::new(m.event.step.value().abs()),
                    noise: m.event.noise,
                })
                .collect();
            match size_with(
                &site.network,
                devices,
                &drops,
                l.most_likely,
                config.speed_margin,
            ) {
                Ok(s) => {
                    if s.misfit > config.max_misfit {
                        reasons.push(format!(
                            "the loggers' drops disagree on the size (misfit {:.1}): an echo or \
                             a second event in the window - size uncertain",
                            s.misfit
                        ));
                    }
                    Some(s)
                }
                Err(e) => {
                    reasons.push(format!("not sized: {e}"));
                    None
                }
            }
        }
        _ => None,
    };
    Alert {
        id,
        revision,
        level,
        class,
        confidence,
        time: Time::new(time.value()),
        location,
        size,
        sensors: members
            .iter()
            .map(|m| site.sensors[m.sensor].name.clone())
            .collect(),
        reasons,
        utc_offset: site.schedule.as_ref().map(|s| s.utc_offset),
    }
}
