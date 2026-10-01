//! The pipe-health demo: a pump station losing power to load-shedding.
//!
//! A sump at 10 m, a pump (65 m at 100 l/s) and a 2 km DN300 rising main
//! to a reservoir at 60 m; PN10 pipes. Three synthetic loggers - at the
//! pump's discharge, mid-main and near the reservoir - record one trip
//! (power lost at 3 s, recorded), fed to the live monitor in 1 s chunks.
//! The monitor gives the alert, and counts every swing each logger saw
//! (rainflow) into its fatigue loading. The main stays above atmospheric
//! - fatigue is never counted from a simulated boiling zone (FN-038).

use jk_application::{Alert, Chunk, CollectingSink, Monitor, PipelineConfig, SensorSite, Site};
use jk_classification::Context;
use jk_health::Spectrum;
use jk_hydraulics::pipe::pn;
use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, Pipe, Probe, Pump, PumpCurve, SteadyOptions,
    thorley_inertia,
};
use jk_kernel::random::Rng;
use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, Time, WaveSpeed};

use super::model::RATE;
use super::svg::to_f64;
use crate::sim::logger::{Logger, sample};

/// Swings of this range [m] are the yardstick ...
pub const REFERENCE: f64 = 50.0;
/// ... and this S-N exponent weighs the others [VERIFY per material].
pub const EXPONENT: f64 = 4.0;
/// Trips a day assumed for the yearly figure (two load-shedding slots).
pub const TRIPS_PER_DAY: f64 = 2.0;

/// One logger: its record and the loading counted from it.
pub struct HealthLogger {
    pub name: String,
    pub start: f64,
    pub samples: Vec<f64>,
    pub loading: Spectrum,
}

impl HealthLogger {
    /// Equivalent `REFERENCE` swings in this record.
    pub fn equivalent(&self) -> f64 {
        self.loading.equivalent_cycles(REFERENCE, EXPONENT)
    }

    /// Cycles with ranges in [lo, hi) m.
    pub fn band(&self, lo: f64, hi: f64) -> f64 {
        self.loading
            .bins()
            .filter(|(r, _)| *r >= lo && *r < hi)
            .map(|(_, n)| n)
            .fold(0.0, |a, n| a + n)
    }
}

/// The demo, run.
pub struct HealthModel {
    pub loggers: Vec<HealthLogger>,
    pub alerts: Vec<Alert>,
    /// The pipes' rating [m].
    pub rating: f64,
    /// Seconds recorded.
    pub duration: f64,
}

impl HealthModel {
    /// Simulates the trip, records it and runs the live engine.
    ///
    /// # Errors
    /// Only if the network, the run or the pipeline fails (a bug).
    #[allow(clippy::too_many_lines)] // one demo: network, loggers, engine
    pub fn run() -> Result<Self, String> {
        let err = |e: jk_hydraulics::HydraulicsError| e.to_string();
        let rating = pn(10.0);
        let pipe = |l: f64| -> Result<Pipe, String> {
            Pipe::new(
                Length::new(l),
                Length::new(0.3),
                WaveSpeed::new(1000.0),
                0.018,
            )
            .and_then(|p| p.with_rating(rating))
            .map_err(err)
        };
        let power = 998.2 * STANDARD_GRAVITY.value() * 0.10 * 65.0 / 0.8 / 1000.0;
        let pump = Pump {
            curve: PumpCurve::from_points(
                Head::new(80.0),
                (Flow::new(0.10), Head::new(65.0)),
                (Flow::new(0.15), Head::new(45.0)),
            )
            .map_err(err)?,
            rated_flow: Flow::new(0.10),
            rated_efficiency: 0.8,
            rated_speed_rpm: 1450.0,
            inertia: thorley_inertia(power, 1450.0),
        };
        let mut net = Network::new(STANDARD_GRAVITY);
        let j = || Node::Junction { demand: Flow::ZERO };
        let sump = net.add_node(Node::Reservoir {
            head: Head::new(10.0),
        });
        let p = net.add_node(Node::Pump {
            pump,
            trip: Some(Time::new(3.0)),
        });
        let mid = net.add_node(j());
        let near = net.add_node(j());
        let top = net.add_node(Node::Reservoir {
            head: Head::new(60.0),
        });
        net.add_pipe(sump, p, pipe(20.0)?).map_err(err)?;
        net.add_pipe(p, mid, pipe(1000.0)?).map_err(err)?;
        net.add_pipe(mid, near, pipe(900.0)?).map_err(err)?;
        net.add_pipe(near, top, pipe(100.0)?).map_err(err)?;
        net.set_vapour_floor(Some(Head::new(-10.1))).map_err(err)?;
        let duration = 40.0;
        let run = net
            .simulate(
                &InitialCondition::Steady(SteadyOptions::default()),
                Time::new(duration),
                &[Probe::Node(p), Probe::Node(mid), Probe::Node(near)],
                &GridOptions {
                    max_wave_speed_adjustment: 0.05,
                    min_reaches: 4,
                },
            )
            .map_err(err)?;
        let dt = run.trace.time_step().value();
        let names = ["L1 pump", "L2 mid-main", "L3 near reservoir"];
        let mut rng = Rng::new(17);
        let recorded: Vec<(f64, Vec<f64>)> = (0..3)
            .map(|k| {
                let truth: Vec<f64> = run.trace.head(k).iter().map(|h| h.value()).collect();
                let logger = Logger {
                    rate: RATE,
                    phase: rng.uniform() / RATE,
                    noise: 0.02,
                };
                (logger.phase, sample(&truth, dt, &logger, &mut rng))
            })
            .collect();
        let site = Site {
            network: net,
            sensors: [p, mid, near]
                .iter()
                .zip(names)
                .map(|(&node, name)| SensorSite {
                    node,
                    name: name.into(),
                    clock: None,
                    uncalibrated_clock: Time::new(1e-6),
                    range: None,
                })
                .collect(),
            pipe_names: vec![
                "suction".into(),
                "main, first km".into(),
                "main, on".into(),
                "main, last 100 m".into(),
            ],
            context: Context::power_lost_at(&[Time::new(3.0)]),
            schedule: None,
            utc_offset: None,
        };
        let mut monitor =
            Monitor::new(site, PipelineConfig::default()).map_err(|e| e.to_string())?;
        let mut sink = CollectingSink::default();
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
                monitor
                    .ingest(sensor, chunk, &mut sink)
                    .map_err(|e| e.to_string())?;
            }
        }
        let loading = monitor.loading();
        let _ = monitor.finish(&mut sink).map_err(|e| e.to_string())?;
        // The latest revision of each alert.
        let mut alerts: Vec<Alert> = Vec::new();
        for a in sink.alerts {
            match alerts.iter_mut().find(|b| b.id == a.id) {
                Some(b) if a.revision >= b.revision => *b = a,
                Some(_) => {}
                None => alerts.push(a),
            }
        }
        let loggers = recorded
            .into_iter()
            .zip(loading)
            .zip(names)
            .map(|(((start, samples), loading), name)| HealthLogger {
                name: name.into(),
                start,
                samples,
                loading,
            })
            .collect();
        Ok(Self {
            loggers,
            alerts,
            rating: rating.value(),
            duration,
        })
    }
}
