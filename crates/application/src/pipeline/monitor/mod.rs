//! The live use case: chunks of sensor data in as they arrive, each alert
//! out once, as soon as the data allows.
//!
//! Each sensor's chunks pass through telemetry (continuity, gaps) into a
//! live detector. A detection is held until its event can be DECIDED:
//!
//! - the record around the first front is in (classification looks 3 s
//!   past it);
//! - every other sensor has reported far enough to say whether the same
//!   wave reached it: a wave from the source reaches sensor j no later than
//!   `t_m + tau_mj (1 + margin) + k sigma_m` for every member m, and j's
//!   detector must be final (its watermark) past that.
//!
//! A sensor that lags more than `max_wait` behind the one furthest ahead
//! (a delayed upload, a gateway off during load-shedding) is not waited
//! for: the alert goes out without it and says so (P6). If its data then
//! arrives and it saw the event too, the alert is REVISED - same id,
//! revision + 1, located again with the extra arrival.
//!
//! Memory is bounded: detectors keep only recent samples, the monitor only
//! recent record for classification and events recent enough to revise.

mod hours;
mod sensor;

use jk_classification::extract;
use jk_detection::{Detector, Event, StreamOutput};
use jk_kernel::Time;
use jk_localisation::ClockEstimate;
use jk_localisation::clocks::correct;
use jk_localisation::graph::TravelGraph;
use jk_telemetry::{Assembler, Chunk, Piece, TelemetryError};

use self::hours::Hours;
use self::sensor::SensorState;
use super::assess::{Found, Unheard, assess};
use super::fusion::{self, Candidate};
use super::{PipelineConfig, Site};
use crate::alert::Alert;
use crate::error::ApplicationError;
use crate::ports::AlertSink;

/// Events are kept for revision, and impaired stretches for corroboration,
/// this long beyond `max_wait` [s].
const KEEP_BEYOND_WAIT: f64 = 60.0;

/// An event already reported.
#[derive(Debug, Clone)]
struct Issued {
    id: u32,
    revision: u32,
    members: Vec<Found>,
}

/// Watches one site live.
#[derive(Debug, Clone)]
pub struct Monitor {
    site: Site,
    config: PipelineConfig,
    detector: Detector,
    /// Fastest wave travel time between every pair of sensors [s], with
    /// the devices as at base demand (for waiting and fusion).
    travel: Vec<Vec<f64>>,
    /// Pumps and valves hour by hour (for locating and corroborating).
    hours: Hours,
    clocks: Vec<ClockEstimate>,
    sensors: Vec<SensorState>,
    /// Detections not yet in a reported event.
    pending: Vec<Found>,
    issued: Vec<Issued>,
    next_id: u32,
}

impl Monitor {
    /// A monitor for `site`.
    ///
    /// # Errors
    /// A site without sensors, a sensor on a missing node, or detector
    /// settings out of range.
    pub fn new(site: Site, config: PipelineConfig) -> Result<Self, ApplicationError> {
        if site.sensors.is_empty() {
            return Err(ApplicationError::InvalidInput(
                "the site has no sensors".into(),
            ));
        }
        for s in &site.sensors {
            if s.node.index() >= site.network.nodes().len() {
                return Err(ApplicationError::InvalidInput(format!(
                    "sensor {} sits on node {} which does not exist",
                    s.name,
                    s.node.index()
                )));
            }
        }
        // A sensor counts as quiet again only after a stretch at least one
        // full echo round trip of THIS network long - otherwise a lull
        // between echoes looks like calm.
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
        let detector =
            Detector::new(detector_config).map_err(|error| ApplicationError::Detection {
                sensor: "(configuration)".into(),
                error,
            })?;
        let hours = Hours::of(&site)?;
        let travel = hours.travel[0].clone();
        let clocks = site.sensors.iter().filter_map(|s| s.clock).collect();
        let sensors = vec![SensorState::default(); site.sensors.len()];
        Ok(Self {
            site,
            config,
            detector,
            travel,
            hours,
            clocks,
            sensors,
            pending: Vec::new(),
            issued: Vec::new(),
            next_id: 1,
        })
    }

    /// The site being watched.
    #[must_use]
    pub fn site(&self) -> &Site {
        &self.site
    }

    /// Every detection the monitor still holds - waiting to be decided, or
    /// part of an event kept for revision - as (sensor number, event on the
    /// sensor's clock), in order of arrival.
    #[must_use]
    pub fn detections(&self) -> Vec<(usize, Event)> {
        let mut all: Vec<(usize, Event)> = self
            .issued
            .iter()
            .flat_map(|g| g.members.iter())
            .chain(&self.pending)
            .map(|f| (f.sensor, f.event))
            .collect();
        all.sort_by(|a, b| a.1.arrival.value().total_cmp(&b.1.arrival.value()));
        all
    }

    /// When sensor number `sensor` was not watching at full sensitivity -
    /// gaps in its data, calibrating, blind after its own events, listening
    /// through ringing - as far as the monitor still remembers; a stretch
    /// still going on runs to its last sample. Empty for an unknown sensor.
    #[must_use]
    pub fn impaired(&self, sensor: usize) -> Vec<(Time, Time)> {
        self.sensors.get(sensor).map_or_else(Vec::new, |s| {
            s.impaired_spans()
                .into_iter()
                .map(|(a, b)| (Time::new(a), Time::new(b)))
                .collect()
        })
    }

    /// Time of the latest sample received from any sensor.
    #[must_use]
    pub fn lead(&self) -> Option<Time> {
        self.sensors
            .iter()
            .filter_map(SensorState::received_until)
            .reduce(f64::max)
            .map(Time::new)
    }

    /// Takes the next chunk of data from sensor number `sensor`, and sends
    /// (and returns) every alert that can now be decided.
    ///
    /// # Errors
    /// An unknown sensor, a chunk refused by telemetry (overlap, changed
    /// rate), or an alert that could not be delivered.
    pub fn ingest(
        &mut self,
        sensor: usize,
        chunk: Chunk<'_>,
        sink: &mut dyn AlertSink,
    ) -> Result<Vec<Alert>, ApplicationError> {
        let Some(site_sensor) = self.site.sensors.get(sensor) else {
            return Err(ApplicationError::InvalidInput(format!(
                "data for sensor {sensor}, which does not exist"
            )));
        };
        let name = site_sensor.name.clone();
        let refused = |error: TelemetryError| ApplicationError::Telemetry {
            sensor: name.clone(),
            error,
        };
        let state = &mut self.sensors[sensor];
        let assembler = match state.assembler.as_mut() {
            Some(a) => a,
            None => state
                .assembler
                .insert(Assembler::new(chunk.interval).map_err(refused)?),
        };
        let pieces = assembler.accept(chunk).map_err(refused)?;
        let interval = assembler.interval();
        for piece in pieces {
            match piece {
                Piece::Gap(gap) => {
                    self.end_stretch(sensor);
                    self.sensors[sensor]
                        .impaired
                        .push((gap.from.value(), gap.to.value()));
                }
                Piece::Samples {
                    start,
                    samples,
                    new_stretch,
                } => {
                    if new_stretch {
                        self.end_stretch(sensor);
                        let stream = self.detector.stream(start, interval).map_err(|error| {
                            ApplicationError::Detection {
                                sensor: name.clone(),
                                error,
                            }
                        })?;
                        let state = &mut self.sensors[sensor];
                        state.stream = Some(stream);
                        state.start_stretch(start.value());
                    }
                    let state = &mut self.sensors[sensor];
                    state.append(samples);
                    let out = match state.stream.as_mut() {
                        Some(s) => {
                            s.push(samples)
                                .map_err(|error| ApplicationError::Detection {
                                    sensor: name.clone(),
                                    error,
                                })?
                        }
                        None => StreamOutput::default(),
                    };
                    self.absorb(sensor, out);
                }
            }
        }
        // Invalid samples still arriving: the stretch has ended.
        if self.sensors[sensor]
            .assembler
            .as_ref()
            .is_some_and(|a| a.open_gap().is_some())
        {
            self.end_stretch(sensor);
        }
        self.update_features(sensor, false);
        self.decide(sink, false)
    }

    /// No more data will come: decides everything still pending.
    ///
    /// # Errors
    /// An alert that could not be delivered.
    pub fn finish(mut self, sink: &mut dyn AlertSink) -> Result<Vec<Alert>, ApplicationError> {
        for s in 0..self.sensors.len() {
            self.end_stretch(s);
            self.update_features(s, true);
        }
        self.decide(sink, true)
    }

    /// Ends the sensor's current continuous stretch, if any.
    fn end_stretch(&mut self, sensor: usize) {
        if let Some(stream) = self.sensors[sensor].stream.take() {
            let out = stream.finish();
            self.absorb(sensor, out);
            self.update_features(sensor, true);
        }
    }

    /// Takes in what a detector produced.
    fn absorb(&mut self, sensor: usize, out: StreamOutput) {
        let state = &mut self.sensors[sensor];
        for s in out.impaired {
            state.impaired.push((s.from.value(), s.to.value()));
        }
        let site_sensor = &self.site.sensors[sensor];
        for event in out.events {
            let arrival = correct(
                site_sensor.node,
                event.arrival,
                event.arrival_uncertainty,
                &self.clocks,
                site_sensor.uncalibrated_clock,
            );
            self.pending.push(Found {
                sensor,
                event,
                arrival,
                features: None,
            });
        }
    }

    /// Measures classification features of the sensor's detections whose
    /// record is complete (or, if `ended`, as complete as it will get).
    fn update_features(&mut self, sensor: usize, ended: bool) {
        let state = &self.sensors[sensor];
        let (Some(dt), Some(until)) = (state.interval(), state.received_until()) else {
            return;
        };
        let f = &self.config.features;
        let before = f.before.value() + 2.0 * dt;
        let after = f.after_window.value().max(f.front_window.value()) + 2.0 * dt;
        let series = state.history(dt);
        let mut keep_from = until - before - after;
        for found in self
            .pending
            .iter_mut()
            .filter(|p| p.sensor == sensor && p.features.is_none())
        {
            if ended || until >= found.event.arrival.value() + after {
                found.features = Some(
                    extract(&series, &found.event, found.event.noise, f).map_err(|e| e.to_string()),
                );
            } else {
                keep_from = keep_from.min(found.event.arrival.value() - before);
            }
        }
        self.sensors[sensor].forget_before(keep_from, dt);
    }

    /// Decides every event the data now allows: first revisions of events
    /// already reported, then new events in time order.
    fn decide(
        &mut self,
        sink: &mut dyn AlertSink,
        all_final: bool,
    ) -> Result<Vec<Alert>, ApplicationError> {
        let mut alerts = Vec::new();

        // Late detections of events already reported: revise them.
        let mut i = 0;
        while i < self.pending.len() {
            let ready = all_final || self.pending[i].features.is_some();
            let home = if ready {
                self.issued
                    .iter()
                    .position(|g| self.fits(&g.members, &self.pending[i]))
            } else {
                None
            };
            let Some(g) = home else {
                i += 1;
                continue;
            };
            let found = self.pending.remove(i);
            let issued = &mut self.issued[g];
            issued.members.push(found);
            sort_by_arrival(&mut issued.members);
            issued.revision += 1;
            let (id, revision) = (issued.id, issued.revision);
            let alert = self.alert(&self.issued[g].members, id, revision);
            sink.send(&alert).map_err(ApplicationError::Delivery)?;
            alerts.push(alert);
        }

        // New events, earliest first.
        while !self.pending.is_empty() {
            let candidates: Vec<Candidate> = self.pending.iter().map(candidate).collect();
            let groups = fusion::group(
                &candidates,
                &self.travel,
                self.config.speed_margin,
                self.config.gate_sigmas,
            );
            let group = &groups[0];
            if !all_final && self.pending[group[0]].features.is_none() {
                break;
            }
            let members: Vec<Found> = group.iter().map(|&k| self.pending[k].clone()).collect();
            let lead = self.lead().map_or(f64::NEG_INFINITY, Time::value);
            let waiting = !all_final
                && self
                    .missing(&members)
                    .iter()
                    .any(|&(_, deadline)| lead - deadline <= self.config.max_wait.value());
            if waiting {
                break;
            }
            let mut taken = group.clone();
            taken.sort_unstable_by(|a, b| b.cmp(a));
            let mut members: Vec<Found> = taken.iter().map(|&k| self.pending.remove(k)).collect();
            sort_by_arrival(&mut members);
            let id = self.next_id;
            self.next_id += 1;
            let alert = self.alert(&members, id, 0);
            sink.send(&alert).map_err(ApplicationError::Delivery)?;
            alerts.push(alert);
            self.issued.push(Issued {
                id,
                revision: 0,
                members,
            });
        }

        self.forget_old();
        Ok(alerts)
    }

    /// The alert for an event, as the data stands.
    fn alert(&self, members: &[Found], id: u32, revision: u32) -> Alert {
        let unheard: Vec<Unheard> = self
            .missing(members)
            .iter()
            .map(|&(j, _)| (j, self.sensors[j].received_until().map(Time::new)))
            .collect();
        let (set, words) = self
            .hours
            .at(self.site.schedule.as_ref(), members[0].arrival.time.value());
        assess(
            &self.site,
            &self.config,
            (&self.hours.sets[set], words),
            &self.hours.travel[set],
            members,
            &|j, from, to| self.sensors[j].watched(from, to),
            &unheard,
            id,
            revision,
        )
    }

    /// Sensors not among `members` whose data is not yet final up to the
    /// latest moment the event's wave could reach them, with that moment.
    fn missing(&self, members: &[Found]) -> Vec<(usize, f64)> {
        (0..self.sensors.len())
            .filter(|&j| members.iter().all(|m| m.sensor != j))
            .filter_map(|j| {
                let deadline = members
                    .iter()
                    .map(|m| {
                        m.arrival.time.value()
                            + self.travel[m.sensor][j] * (1.0 + self.config.speed_margin)
                            + self.config.gate_sigmas * m.arrival.uncertainty.value()
                    })
                    .fold(f64::INFINITY, f64::min);
                (deadline.is_finite() && self.sensors[j].final_until() < deadline)
                    .then_some((j, deadline))
            })
            .collect()
    }

    /// Whether a detection can join an event already reported.
    fn fits(&self, members: &[Found], found: &Found) -> bool {
        let c = candidate(found);
        members.iter().all(|m| {
            m.sensor != found.sensor
                && fusion::consistent(
                    &candidate(m),
                    &c,
                    &self.travel,
                    self.config.speed_margin,
                    self.config.gate_sigmas,
                )
        })
    }

    /// Forgets events too old to revise and impaired stretches too old to
    /// matter.
    fn forget_old(&mut self) {
        let Some(lead) = self.lead() else { return };
        let horizon = lead.value() - self.config.max_wait.value() - KEEP_BEYOND_WAIT;
        if !horizon.is_finite() {
            return;
        }
        self.issued
            .retain(|g| g.members.iter().any(|m| m.arrival.time.value() >= horizon));
        for s in &mut self.sensors {
            s.forget_impaired_before(horizon);
        }
    }
}

fn candidate(f: &Found) -> Candidate {
    Candidate {
        sensor: f.sensor,
        time: f.arrival.time.value(),
        sigma: f.arrival.uncertainty.value(),
    }
}

fn sort_by_arrival(members: &mut [Found]) {
    members.sort_by(|a, b| a.arrival.time.value().total_cmp(&b.arrival.time.value()));
}

/// The longest fastest wave travel time between any two nodes [s].
fn network_diameter(graph: &TravelGraph, pipe_times: &[f64]) -> f64 {
    (0..graph.len())
        .flat_map(|n| graph.times_from(n, pipe_times))
        .filter(|t| t.is_finite())
        .fold(0.0, f64::max)
}
