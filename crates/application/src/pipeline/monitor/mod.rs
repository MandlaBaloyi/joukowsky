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
mod supply;

use jk_classification::EventClass;
use jk_classification::extract;
use jk_detection::{Detector, Direction, Event, StreamOutput, SupplyChange, SupplyWatch};
use jk_health::{Counter, Spectrum};
use jk_kernel::clock::stamp;
use jk_kernel::{Head, Time};
use jk_localisation::ClockEstimate;
use jk_localisation::clocks::correct;
use jk_localisation::graph::TravelGraph;
use jk_telemetry::{Assembler, Chunk, Piece, TelemetryError};

use self::hours::Hours;
use self::sensor::SensorState;
use self::supply::{Incident, Member};
use super::assess::{Found, Unheard, assess};
use super::fusion::{self, Candidate};
use super::{PipelineConfig, Site};
use crate::alert::{Alert, Level};
use crate::error::ApplicationError;
use crate::ports::AlertSink;

/// Events are kept for revision, and impaired stretches for corroboration,
/// this long beyond `max_wait` [s].
const KEEP_BEYOND_WAIT: f64 = 60.0;

/// A drop up to this long before a pipe stood empty may be the drain that
/// began a supply cut [s]; issued alerts are kept at least this long, so a
/// cut can be tied to it.
const DRAIN: f64 = 600.0;

/// An event already reported.
#[derive(Debug, Clone)]
struct Issued {
    id: u32,
    revision: u32,
    members: Vec<Found>,
    /// A late sensor brought low-pressure news: revise now.
    stale: bool,
    /// A follow-up peak was measured: revise once all are in.
    followed: bool,
    /// Notes from other alerts (a supply cut it led into).
    links: Vec<String>,
    /// Explained by a supply cut: an unknown drop is then Info.
    explained: bool,
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
    /// Whether each sensor's node touches a pipe with a pressure rating:
    /// its events are followed up for their peak (it may come later, in
    /// the ringing).
    rated: Vec<bool>,
    /// Pumps and valves hour by hour (for locating and corroborating).
    hours: Hours,
    clocks: Vec<ClockEstimate>,
    sensors: Vec<SensorState>,
    /// Detections not yet in a reported event.
    pending: Vec<Found>,
    issued: Vec<Issued>,
    /// Supply cuts, open or recent.
    supply_cuts: Vec<Incident>,
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
        let rated = site
            .sensors
            .iter()
            .map(|s| {
                site.network
                    .pipes()
                    .iter()
                    .any(|p| (p.from == s.node || p.to == s.node) && p.pipe.rating().is_some())
            })
            .collect();
        let hours = Hours::of(&site)?;
        let travel = hours.travel[0].clone();
        let clocks = site.sensors.iter().filter_map(|s| s.clock).collect();
        let loading = Spectrum::new(config.fatigue_bin.value())
            .map_err(|e| ApplicationError::InvalidInput(format!("fatigue bin: {e}")))?;
        let mut fresh = SensorState::default();
        fresh.loading = Some(loading);
        let sensors = vec![fresh; site.sensors.len()];
        Ok(Self {
            site,
            config,
            detector,
            travel,
            rated,
            hours,
            clocks,
            sensors,
            pending: Vec::new(),
            issued: Vec::new(),
            supply_cuts: Vec::new(),
            next_id: 1,
        })
    }

    /// The fatigue loading each sensor's pipes have taken so far: every
    /// swing counted (rainflow, gated at `fatigue_gate`), in sensor order.
    /// Swings still open - a fading ring closes none - count as half
    /// cycles, as the end of the record would count them.
    #[must_use]
    pub fn loading(&self) -> Vec<Spectrum> {
        self.sensors
            .iter()
            .filter_map(|s| {
                let mut spectrum = s.loading.clone()?;
                if let Some(counter) = &s.rainflow {
                    spectrum.add(&counter.open());
                }
                Some(spectrum)
            })
            .collect()
    }

    /// The sensors ranked by fatigue loading: the cycles of
    /// `reference_range` [m] doing the same damage for S-N exponent
    /// `exponent` (`jk_health::damage`), largest first, with names.
    #[must_use]
    pub fn loading_ranking(&self, reference_range: f64, exponent: f64) -> Vec<(String, f64)> {
        let mut ranked: Vec<(String, f64)> = self
            .site
            .sensors
            .iter()
            .zip(self.loading())
            .map(|(s, l)| {
                (
                    s.name.clone(),
                    l.equivalent_cycles(reference_range, exponent),
                )
            })
            .collect();
        ranked.sort_by(|a, b| b.1.total_cmp(&a.1));
        ranked
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
    #[allow(clippy::too_many_lines)] // one chunk through every watch, in order
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
                        let supply = SupplyWatch::new(self.config.supply, start, interval)
                            .map_err(|error| ApplicationError::Detection {
                                sensor: name.clone(),
                                error,
                            })?;
                        let counter =
                            Counter::new(self.config.fatigue_gate.value()).map_err(|e| {
                                ApplicationError::InvalidInput(format!("fatigue gate: {e}"))
                            })?;
                        let state = &mut self.sensors[sensor];
                        state.stream = Some(stream);
                        state.supply = Some(supply);
                        state.rainflow = Some(counter);
                        state.start_stretch(start.value());
                    }
                    let state = &mut self.sensors[sensor];
                    state.append(samples);
                    if let (Some(counter), Some(loading)) =
                        (state.rainflow.as_mut(), state.loading.as_mut())
                    {
                        counter
                            .push(samples)
                            .map_err(|e| ApplicationError::InvalidInput(format!("{name}: {e}")))?;
                        loading.add(&counter.take_closed());
                    }
                    let mut changes = Vec::new();
                    if let Some(watch) = state.supply.as_mut() {
                        changes =
                            watch
                                .push(samples)
                                .map_err(|error| ApplicationError::Detection {
                                    sensor: name.clone(),
                                    error,
                                })?;
                        state
                            .supply_returns
                            .extend(changes.iter().filter_map(|c| match c {
                                SupplyChange::Returned(s) => s.until.map(Time::value),
                                SupplyChange::Emptied { .. } => None,
                            }));
                    }
                    self.supply_changes(sensor, &changes);
                    let state = &mut self.sensors[sensor];
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
        // A spell still open at a gap: whether the supply came back during
        // it is not known (P6) - and a cut it was part of says so.
        if self.sensors[sensor].supply.take().is_some() {
            let stop = self.sensors[sensor].received_until();
            for incident in &mut self.supply_cuts {
                for m in incident
                    .members
                    .iter_mut()
                    .filter(|m| m.sensor == sensor && m.open())
                {
                    let at = stop.unwrap_or(m.confirmed);
                    m.data_stopped = Some(at);
                    incident.stale = true;
                    incident.changed_at = incident.changed_at.max(at);
                }
            }
        }
        // A swing across a gap is not known either: what is still open
        // counts as half cycles.
        let state = &mut self.sensors[sensor];
        if let (Some(counter), Some(loading)) = (state.rainflow.take(), state.loading.as_mut()) {
            loading.add(&counter.finish());
        }
        if let Some(stream) = self.sensors[sensor].stream.take() {
            let out = stream.finish();
            self.absorb(sensor, out);
            self.update_features(sensor, true);
        }
    }

    /// Ties a supply cut (incident `i`) to the drop that began it at
    /// `sensor`, whose pipe stood empty from `start` [s]: an alert already
    /// out for a drop there up to `DRAIN` before. Both say so; an UNKNOWN
    /// drop is explained (Info) - a BURST is never relabelled, as a burst
    /// can itself drain the pipes.
    fn link_drain(&mut self, i: usize, sensor: usize, start: f64) {
        let zone = self.site.zone();
        let name = self.site.sensors[sensor].name.clone();
        let cut_id = self.supply_cuts[i].id;
        for g in 0..self.issued.len() {
            let Some(arrival) = self.issued[g]
                .members
                .iter()
                .find(|m| {
                    let t = m.event.arrival.value();
                    m.sensor == sensor
                        && m.event.direction == Direction::Drop
                        && (start - DRAIN..=start + 1.0).contains(&t)
                })
                .map(|m| m.event.arrival.value())
            else {
                continue;
            };
            let id = self.issued[g].id;
            if self.supply_cuts[i].linked.contains(&id) {
                continue;
            }
            let class = self
                .alert(&self.issued[g].members, id, self.issued[g].revision)
                .class;
            let ahead = start - arrival;
            self.supply_cuts[i].linked.push(id);
            self.supply_cuts[i].links.push(format!(
                "{name}: the pressure began falling at {} (alert #{id}, {class}), {ahead:.0} s before \
                 the pipe stood empty{}",
                stamp(arrival, zone),
                if class == EventClass::Burst {
                    " - the burst may have drained the pipes"
                } else {
                    ""
                }
            ));
            let issued = &mut self.issued[g];
            issued.links.push(format!(
                "followed by a SUPPLY CUT (alert #{cut_id}): the pipe at {name} stood empty from {}",
                stamp(start, zone)
            ));
            if class == EventClass::Unknown {
                issued.explained = true;
                issued.links.push(
                    "this drop was most likely the pressure draining at the start of that cut, \
                     not a burst"
                        .into(),
                );
            }
            issued.stale = true;
        }
    }

    /// Sends the supply cuts that changed: raised when confirmed, revised
    /// as they change. Closed ones are forgotten after a day.
    fn send_supply_cuts(
        &mut self,
        sink: &mut dyn AlertSink,
        all_final: bool,
    ) -> Result<Vec<Alert>, ApplicationError> {
        let names: Vec<String> = self.site.sensors.iter().map(|s| s.name.clone()).collect();
        let offset = self.site.zone();
        let lead = self.lead().map_or(f64::NEG_INFINITY, Time::value);
        let reached: Vec<f64> = self
            .sensors
            .iter()
            .map(|s| s.received_until().unwrap_or(f64::NEG_INFINITY))
            .collect();
        let max_wait = self.config.max_wait.value();
        let mut alerts = Vec::new();
        for incident in &mut self.supply_cuts {
            // Every sensor past the change - or lagging beyond the wait.
            let heard = reached
                .iter()
                .all(|&r| r >= incident.changed_at || lead - r > max_wait);
            if incident.stale && (heard || all_final) {
                incident.stale = false;
                let alert = incident.alert(&names, offset);
                incident.revision += 1;
                sink.send(&alert).map_err(ApplicationError::Delivery)?;
                alerts.push(alert);
            }
        }
        if let Some(lead) = self.lead() {
            self.supply_cuts.retain(|i| {
                i.members.iter().any(Member::open)
                    || i.members
                        .iter()
                        .filter_map(|m| m.until.or(m.data_stopped))
                        .any(|t| lead.value() - t < 86_400.0)
            });
        }
        Ok(alerts)
    }

    /// Takes in what a sensor's supply watch noticed: a confirmed cut joins
    /// (or opens) an incident; a return closes the sensor's part of it.
    fn supply_changes(&mut self, sensor: usize, changes: &[SupplyChange]) {
        for change in changes {
            match *change {
                SupplyChange::Emptied { from, confirmed } => {
                    let member = Member {
                        sensor,
                        from: from.map(Time::value),
                        confirmed: confirmed.value(),
                        until: None,
                        data_stopped: None,
                    };
                    let home = self
                        .supply_cuts
                        .iter()
                        .position(|i| i.takes(member.confirmed));
                    let at = member.confirmed;
                    let start = member.from;
                    let i = if let Some(i) = home {
                        let incident = &mut self.supply_cuts[i];
                        incident.members.push(member);
                        incident.stale = true;
                        incident.changed_at = incident.changed_at.max(at);
                        i
                    } else {
                        self.supply_cuts.push(Incident {
                            id: self.next_id,
                            revision: 0,
                            members: vec![member],
                            stale: true,
                            changed_at: at,
                            links: Vec::new(),
                            linked: Vec::new(),
                        });
                        self.next_id += 1;
                        self.supply_cuts.len() - 1
                    };
                    if let Some(start) = start {
                        self.link_drain(i, sensor, start);
                    }
                }
                SupplyChange::Returned(spell) => {
                    for incident in &mut self.supply_cuts {
                        for m in incident
                            .members
                            .iter_mut()
                            .filter(|m| m.sensor == sensor && m.open())
                        {
                            m.until = spell.until.map(Time::value);
                            incident.stale = true;
                            if let Some(back) = m.until {
                                incident.changed_at = incident.changed_at.max(back);
                            }
                        }
                    }
                }
            }
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
                supply_return: None,
                peak_after: None,
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
        // Measures a detection's features once the record around it is in;
        // false if it must wait.
        let measure = |found: &mut Found| {
            if !(ended || until >= found.event.arrival.value() + after) {
                return false;
            }
            // The clock correction applies to the return as to the arrival.
            let shift = found.arrival.time.value() - found.event.arrival.value();
            found.supply_return = state
                .supply_returned_by(found.event.arrival.value())
                .map(|back| Time::new(back + shift));
            found.features = Some(
                extract(&series, &found.event, found.event.noise, f).map_err(|e| e.to_string()),
            );
            true
        };
        for found in self
            .pending
            .iter_mut()
            .filter(|p| p.sensor == sensor && p.features.is_none())
        {
            if !measure(found) {
                keep_from = keep_from.min(found.event.arrival.value() - before);
            }
        }
        // An alert goes out once its FIRST detection is measured; the
        // others are measured when their record is in. What they say about
        // low pressure revises the alert.
        for issued in &mut self.issued {
            for found in issued
                .members
                .iter_mut()
                .filter(|m| m.sensor == sensor && m.features.is_none())
            {
                if measure(found) {
                    issued.stale |= found.wants_follow_up();
                } else {
                    keep_from = keep_from.min(found.event.arrival.value() - before);
                }
            }
        }
        // Follow-ups: events that took the pressure to or below
        // atmospheric are watched on for the collapse that may follow.
        let follow = self.config.follow_up.value();
        let peak = |found: &Found| {
            let from = found.event.arrival.value();
            let span = (until - from).min(follow).max(0.0);
            let first = ((from - series.start.value()) / dt).floor().max(0.0);
            let last = ((from + span - series.start.value()) / dt).ceil();
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // >= 0
            let (first, last) = (
                first as usize,
                (last.max(0.0) as usize).min(series.samples.len()),
            );
            let highest = series.samples[first.min(last)..last]
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            highest
                .is_finite()
                .then(|| (Head::new(highest), Time::new(span)))
        };
        let rated = self.rated[sensor];
        let due = |found: &Found| {
            found.sensor == sensor
                && found.peak_after.is_none()
                && (found.wants_follow_up() || (rated && matches!(found.features, Some(Ok(_)))))
        };
        for found in self.pending.iter_mut().filter(|f| due(f)) {
            if ended || until >= found.event.arrival.value() + follow {
                found.peak_after = peak(found);
            } else {
                keep_from = keep_from.min(found.event.arrival.value() - before);
            }
        }
        for issued in &mut self.issued {
            for found in issued.members.iter_mut().filter(|f| due(f)) {
                if ended || until >= found.event.arrival.value() + follow {
                    found.peak_after = peak(found);
                    issued.followed |= found.peak_after.is_some();
                } else {
                    keep_from = keep_from.min(found.event.arrival.value() - before);
                }
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

        alerts.extend(self.send_supply_cuts(sink, all_final)?);

        // News since: low pressure from a late sensor at once; follow-up
        // peaks together, once every member's is in (one revision).
        for g in 0..self.issued.len() {
            let pending = self.issued[g]
                .members
                .iter()
                .any(|m| self.follow_up_pending(m));
            let issued = &self.issued[g];
            if issued.stale || (issued.followed && (!pending || all_final)) {
                self.issued[g].stale = false;
                self.issued[g].followed = false;
                self.issued[g].revision += 1;
                let alert = self.alert_of(g);
                sink.send(&alert).map_err(ApplicationError::Delivery)?;
                alerts.push(alert);
            }
        }

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
            let alert = self.alert_of(g);
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
                stale: false,
                followed: false,
                links: Vec::new(),
                explained: false,
            });
        }

        self.forget_old();
        Ok(alerts)
    }

    /// The alert for an event, as the data stands.
    /// The alert for issued event `g`, with what other alerts said of it.
    fn alert_of(&self, g: usize) -> Alert {
        let issued = &self.issued[g];
        let mut alert = self.alert(&issued.members, issued.id, issued.revision);
        alert.reasons.extend(issued.links.iter().cloned());
        if issued.explained && alert.class == EventClass::Unknown {
            alert.level = Level::Info;
        }
        alert
    }

    fn alert(&self, members: &[Found], id: u32, revision: u32) -> Alert {
        let unheard: Vec<Unheard> = self
            .missing(members)
            .iter()
            .map(|&(j, _)| (j, self.sensors[j].received_until().map(Time::new)))
            .collect();
        let (set, words) = self.hours.at(
            self.site.schedule.as_ref(),
            self.site.zone(),
            members[0].arrival.time.value(),
        );
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

    /// Whether a detection still waits for its follow-up peak.
    fn follow_up_pending(&self, found: &Found) -> bool {
        found.peak_after.is_none()
            && match &found.features {
                Some(Ok(_)) => found.wants_follow_up() || self.rated[found.sensor],
                Some(Err(_)) => false,
                None => true,
            }
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
        // Issued alerts also wait for a supply cut their drop may begin.
        let keep_issued = horizon.min(lead.value() - DRAIN - self.config.supply.min_empty.value());
        self.issued.retain(|g| {
            g.members
                .iter()
                .any(|m| m.arrival.time.value() >= keep_issued)
        });
        let refill_window = self.config.classifier.refill_window.value();
        for s in &mut self.sensors {
            s.forget_impaired_before(horizon);
            s.supply_returns
                .retain(|&back| back >= horizon - refill_window);
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
