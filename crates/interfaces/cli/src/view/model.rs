//! What the viewer shows, computed once: the engine run on the demo sites.
//!
//! [`EventModel`]: the flowing estate with two bursts, recorded by three
//! synthetic loggers and fed to the real live [`Monitor`] in 1-second
//! chunks, as a site would send them. [`PlacementModel`]: the placement
//! advisor on the campus. The pages only read these.

use jk_application::{Alert, Chunk, CollectingSink, Monitor, PipelineConfig, SensorSite, Site};
use jk_classification::Context;
use jk_hydraulics::NodeId;
use jk_kernel::Time;
use jk_kernel::random::Rng;
use jk_localisation::placement::{Advice, Detectability, PlacementOptions, advise, detectability};

use super::svg::to_f64;
use crate::sim::logger::{Logger, sample};
use crate::sim::sites::{BurstDemo, DemoSite, campus, flowing_estate_two_bursts};

/// Logger rate [S/s].
pub const RATE: f64 = 128.0;

/// An alert and how far the data had come when it went out.
pub struct Issued {
    pub alert: Alert,
    /// Time of the latest sample received when it went out [s].
    pub data_time: f64,
}

/// A front a logger's detector picked.
#[derive(Debug, Clone, Copy)]
pub struct Front {
    /// Arrival time on the logger's clock [s].
    pub arrival: f64,
    /// Size of the step: new level minus old [m].
    pub step: f64,
}

/// One logger's record and what its detector made of it.
pub struct Record {
    pub name: String,
    /// Time of the first sample [s].
    pub start: f64,
    /// Sampling interval [s].
    pub interval: f64,
    /// Heads [m].
    pub samples: Vec<f64>,
    pub fronts: Vec<Front>,
    /// When it was not watching at full sensitivity [s].
    pub impaired: Vec<(f64, f64)>,
}

impl Record {
    /// Time of the last sample [s].
    pub fn end(&self) -> f64 {
        self.start + self.interval * to_f64(self.samples.len().saturating_sub(1))
    }
}

/// The event demo, run.
pub struct EventModel {
    pub demo: BurstDemo,
    pub records: Vec<Record>,
    pub issued: Vec<Issued>,
}

impl EventModel {
    /// Simulates, records, and runs the live engine.
    ///
    /// # Errors
    /// Only if the simulation or the pipeline fails (a bug).
    #[allow(clippy::too_many_lines)] // one demo, set up in order
    pub fn run() -> Result<Self, String> {
        let demo = flowing_estate_two_bursts()?;
        let names: Vec<String> = demo
            .loggers
            .iter()
            .map(|n| demo.site.drawing.node_names[n.index()].clone())
            .collect();
        let mut rng = Rng::new(3);
        let recorded: Vec<(f64, Vec<f64>)> = demo
            .truths
            .iter()
            .map(|truth| {
                let logger = Logger {
                    rate: RATE,
                    phase: rng.uniform() / RATE,
                    noise: 0.02,
                };
                (logger.phase, sample(truth, demo.dt, &logger, &mut rng))
            })
            .collect();

        let site = Site {
            network: demo.site.network.clone(),
            sensors: demo
                .loggers
                .iter()
                .zip(&names)
                .map(|(&node, name)| SensorSite {
                    node,
                    name: name.clone(),
                    clock: None,
                    uncalibrated_clock: Time::new(1e-6),
                    range: None,
                })
                .collect(),
            pipe_names: demo.site.drawing.pipe_names.clone(),
            context: Context::unknown(),
            schedule: None,
            utc_offset: None,
        };
        let mut monitor =
            Monitor::new(site, PipelineConfig::default()).map_err(|e| e.to_string())?;
        let mut sink = CollectingSink::default();
        let mut issued = Vec::new();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 128
        let per = RATE as usize;
        let rounds = recorded
            .iter()
            .map(|(_, x)| x.len().div_ceil(per))
            .max()
            .unwrap_or(0);
        for n in 0..rounds {
            for (sensor, (start, x)) in recorded.iter().enumerate() {
                let from = n * per;
                if from >= x.len() {
                    continue;
                }
                let chunk = Chunk {
                    start: Time::new(start + to_f64(from) / RATE),
                    interval: Time::new(1.0 / RATE),
                    samples: &x[from..(from + per).min(x.len())],
                };
                for alert in monitor
                    .ingest(sensor, chunk, &mut sink)
                    .map_err(|e| e.to_string())?
                {
                    let data_time = monitor.lead().map_or(0.0, Time::value);
                    issued.push(Issued { alert, data_time });
                }
            }
        }
        let detections = monitor.detections();
        let records = recorded
            .into_iter()
            .enumerate()
            .map(|(s, (start, samples))| Record {
                name: names[s].clone(),
                start,
                interval: 1.0 / RATE,
                samples,
                fronts: detections
                    .iter()
                    .filter(|(sensor, _)| *sensor == s)
                    .map(|(_, e)| Front {
                        arrival: e.arrival.value(),
                        step: e.step.value(),
                    })
                    .collect(),
                impaired: monitor
                    .impaired(s)
                    .iter()
                    .map(|(a, b)| (a.value(), b.value()))
                    .collect(),
            })
            .collect();
        let data_time = monitor.lead().map_or(0.0, Time::value);
        for alert in monitor.finish(&mut sink).map_err(|e| e.to_string())? {
            issued.push(Issued { alert, data_time });
        }
        Ok(Self {
            demo,
            records,
            issued,
        })
    }

    /// The front each logger picked for `alert`: its first front from just
    /// before the alert's start to 2 s after (longer than any wave takes to
    /// cross this network).
    pub fn arrivals(&self, alert: &Alert) -> Vec<Option<Front>> {
        let t = alert.time.value();
        self.records
            .iter()
            .map(|r| {
                r.fronts
                    .iter()
                    .copied()
                    .find(|f| f.arrival >= t - 0.05 && f.arrival <= t + 2.0)
            })
            .collect()
    }

    /// Time span of the records [s].
    pub fn span(&self) -> (f64, f64) {
        let end = self.records.iter().map(Record::end).fold(0.0, f64::max);
        (0.0, end.ceil())
    }
}

/// The placement demo, run.
pub struct PlacementModel {
    pub site: DemoSite,
    pub candidates: Vec<NodeId>,
    pub advice: Advice,
    pub felt: Detectability,
}

impl PlacementModel {
    /// Chooses `count` loggers for the campus.
    ///
    /// # Errors
    /// Only if the site or the advisor fails (a bug), or `count` is out of
    /// range.
    pub fn run(count: usize) -> Result<Self, String> {
        let (site, candidates) = campus()?;
        let options = PlacementOptions::default();
        let advice = advise(&site.network, &candidates, count, &[], None, &options)
            .map_err(|e| e.to_string())?;
        let felt = detectability(&site.network, &advice.best.sensors, &options)
            .map_err(|e| e.to_string())?;
        Ok(Self {
            site,
            candidates,
            advice,
            felt,
        })
    }

    /// The largest "smallest locatable burst" over the pipes [m^3/s]: any
    /// sudden burst this big is located anywhere.
    pub fn smallest_anywhere(&self) -> f64 {
        self.felt
            .pipes
            .iter()
            .map(|p| p.smallest_locatable.value())
            .fold(0.0, f64::max)
    }
}
