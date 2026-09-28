//! The use case: one window of records in, alerts out.

use jk_classification::{EventClass, classify, extract};
use jk_detection::{Detector, Event, Span};
use jk_kernel::Time;
use jk_localisation::clocks::correct;
use jk_localisation::graph::TravelGraph;
use jk_localisation::{Arrival, ClockEstimate, localise};

use crate::alert::{Alert, Level};
use crate::error::ApplicationError;
use crate::pipeline::fusion::{self, Candidate};
use crate::pipeline::{PipelineConfig, Record, Site};
use crate::ports::AlertSink;

/// A detection with everything later stages need.
struct Found {
    record: usize,
    event: Event,
    noise: jk_kernel::Head,
    arrival: Arrival,
}

/// Processes one window of `records` from `site`: detect at every sensor,
/// fuse, locate, classify, and hand one alert per event to `sink`. Returns
/// the alerts too.
///
/// # Errors
/// Records that do not match the site, detection or localisation failures,
/// or an alert that could not be delivered.
#[allow(clippy::too_many_lines)] // the use case, told in one place
pub fn process(
    site: &Site,
    records: &[Record<'_>],
    config: &PipelineConfig,
    sink: &mut dyn AlertSink,
) -> Result<Vec<Alert>, ApplicationError> {
    validate(site, records)?;
    let clocks: Vec<ClockEstimate> = site.sensors.iter().filter_map(|s| s.clock).collect();

    // 1. Detection at every sensor. A sensor counts as quiet again only
    //    after a stretch at least one full echo round trip of THIS network
    //    long - otherwise a lull between echoes looks like calm.
    let graph = TravelGraph::new(&site.network);
    let pipe_times: Vec<f64> = site
        .network
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let round_trip = 2.0 * network_diameter(&graph, &pipe_times);
    let mut detector_config = config.detector;
    if detector_config.settle_window.value() < round_trip {
        detector_config.settle_window = Time::new(round_trip);
    }
    let detector = Detector::new(detector_config).map_err(|error| ApplicationError::Detection {
        sensor: "(configuration)".into(),
        error,
    })?;
    let mut found: Vec<Found> = Vec::new();
    let mut impaired: Vec<Vec<Span>> = Vec::new();
    for (r, record) in records.iter().enumerate() {
        let sensor = &site.sensors[record.sensor];
        let detection =
            detector
                .run(&record.series)
                .map_err(|error| ApplicationError::Detection {
                    sensor: sensor.name.clone(),
                    error,
                })?;
        impaired.push(detection.impaired);
        for event in detection.events {
            let arrival = correct(
                sensor.node,
                event.arrival,
                event.arrival_uncertainty,
                &clocks,
                sensor.uncalibrated_clock,
            );
            found.push(Found {
                record: r,
                event,
                noise: detection.noise,
                arrival,
            });
        }
    }

    // 2. Fusion with the physical coincidence gate.
    let travel: Vec<Vec<f64>> = site
        .sensors
        .iter()
        .map(|a| {
            let times = graph.times_from(a.node.index(), &pipe_times);
            site.sensors.iter().map(|b| times[b.node.index()]).collect()
        })
        .collect();
    let candidates: Vec<Candidate> = found
        .iter()
        .map(|f| Candidate {
            sensor: records[f.record].sensor,
            time: f.arrival.time.value(),
            sigma: f.arrival.uncertainty.value(),
        })
        .collect();
    let groups = fusion::group(
        &candidates,
        &travel,
        config.speed_margin,
        config.gate_sigmas,
    );

    // 3 + 4. Locate, classify, alert.
    let mut alerts = Vec::new();
    for (n, members) in groups.iter().enumerate() {
        let first = &found[members[0]];
        let mut reasons = Vec::new();

        // Classify on the first sensor to see it: closest, clearest signature.
        let record = &records[first.record];
        let (class, confidence) =
            match extract(&record.series, &first.event, first.noise, &config.features) {
                Ok(features) => {
                    let c = classify(
                        &features,
                        &site.context,
                        first.arrival.time,
                        &config.classifier,
                    );
                    reasons.extend(c.reasons.iter().cloned());
                    (c.class, c.confidence)
                }
                Err(e) => {
                    reasons.push(format!("not classified: {e}"));
                    (EventClass::Unknown, 0.0)
                }
            };

        // Locate if at least two sensors saw it.
        let arrivals: Vec<Arrival> = members.iter().map(|&m| found[m].arrival).collect();
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
            let me = records[first.record].sensor;
            let t = first.arrival.time.value();
            let watching: Vec<&str> = (0..records.len())
                .filter(|&r| r != first.record)
                .filter(|&r| {
                    let tau = travel[me][records[r].sensor];
                    tau.is_finite() && was_watching(&records[r], &impaired[r], t - tau, t + tau)
                })
                .map(|r| site.sensors[records[r].sensor].name.as_str())
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

        let level = match class {
            _ if !corroborated => Level::Review,
            EventClass::Burst if fits && confidence >= config.alarm_confidence => Level::Alarm,
            EventClass::PumpTrip => Level::Warning,
            EventClass::ValveClosure => Level::Info,
            EventClass::Burst | EventClass::Unknown => Level::Review,
        };
        let time = location.as_ref().map_or(first.arrival.time, |l| l.emission);
        let alert = Alert {
            id: u32::try_from(n + 1).unwrap_or(u32::MAX),
            level,
            class,
            confidence,
            time: Time::new(time.value()),
            location,
            sensors: members
                .iter()
                .map(|&m| site.sensors[records[found[m].record].sensor].name.clone())
                .collect(),
            reasons,
        };
        sink.send(&alert).map_err(ApplicationError::Delivery)?;
        alerts.push(alert);
    }
    Ok(alerts)
}

/// Whether `record`'s sensor covered all of `[from, to]` at full
/// sensitivity (no blind or ringing span of its own overlapping it).
fn was_watching(record: &Record<'_>, impaired: &[Span], from: f64, to: f64) -> bool {
    let series = &record.series;
    let end = series.time_at(to_f64(series.samples.len().saturating_sub(1)));
    from >= series.start.value()
        && to <= end.value()
        && impaired
            .iter()
            .all(|s| s.to.value() < from || s.from.value() > to)
}

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}

/// The longest fastest wave travel time between any two nodes [s].
fn network_diameter(graph: &TravelGraph, pipe_times: &[f64]) -> f64 {
    (0..graph.len())
        .flat_map(|n| graph.times_from(n, pipe_times))
        .filter(|t| t.is_finite())
        .fold(0.0, f64::max)
}

fn validate(site: &Site, records: &[Record<'_>]) -> Result<(), ApplicationError> {
    let bad = |why: String| Err(ApplicationError::InvalidInput(why));
    if site.sensors.is_empty() {
        return bad("the site has no sensors".into());
    }
    for s in &site.sensors {
        if s.node.index() >= site.network.nodes().len() {
            return bad(format!(
                "sensor {} sits on node {} which does not exist",
                s.name,
                s.node.index()
            ));
        }
    }
    for (i, r) in records.iter().enumerate() {
        if r.sensor >= site.sensors.len() {
            return bad(format!(
                "record {i} is for sensor {} which does not exist",
                r.sensor
            ));
        }
        if records[..i].iter().any(|o| o.sensor == r.sensor) {
            return bad(format!(
                "two records for sensor {}",
                site.sensors[r.sensor].name
            ));
        }
    }
    Ok(())
}
