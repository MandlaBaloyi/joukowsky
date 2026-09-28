//! Pipeline tests: sensor records in, alerts out.
//!
//! Code under test: `crates/application` (the event pipeline), driving every
//! domain context. Each scenario simulates events on the estate network,
//! records them with synthetic 128 S/s loggers, and checks the ALERTS - the
//! product's actual output.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only pipeline`

mod simulation;

use jk_application::{Alert, CollectingSink, PipelineConfig, Record, SensorSite, Site, process};
use jk_classification::Context;
use jk_detection::Series;
use jk_hydraulics::{GridOptions, InitialCondition, Network, Probe};
use jk_kernel::Time;
use jk_tests::estate::estate;
use jk_tests::sensor::{Logger, Rng, sample};

/// Logger rate [S/s].
pub const RATE: f64 = 128.0;

/// The site as the engine knows it: the estate's plain pipe map, three
/// GNSS-timed sensors, named pipes.
#[must_use]
pub fn estate_site(context: Context) -> Site {
    let (e, _) = estate(None);
    Site {
        network: e.net,
        sensors: ["S1 gatehouse", "S2 north block", "S3 south block"]
            .iter()
            .zip(e.sensors)
            .map(|(name, node)| SensorSite {
                node,
                name: (*name).to_string(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
            })
            .collect(),
        pipe_names: vec![
            "R-S1 supply main".into(),
            "S1-T spine".into(),
            "T-S2 north branch".into(),
            "T-S3 south branch".into(),
        ],
        context,
    }
}

/// Simulates `net` from `initial` for `duration` s and returns the head
/// history at each of `sensors`.
///
/// # Panics
/// If the simulation fails (a test bug).
#[must_use]
pub fn simulate(
    net: &Network,
    initial: &InitialCondition,
    sensors: &[jk_hydraulics::NodeId],
    duration: f64,
) -> (Vec<Vec<f64>>, f64) {
    let probes: Vec<Probe> = sensors.iter().map(|&n| Probe::Node(n)).collect();
    let run = net
        .simulate(
            initial,
            Time::new(duration),
            &probes,
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .expect("simulation runs");
    let heads = (0..sensors.len())
        .map(|i| run.trace.head(i).iter().map(|h| h.value()).collect())
        .collect();
    (heads, run.trace.time_step().value())
}

/// Records each history with its own 128 S/s logger (random phase, 2 cm
/// noise), runs the pipeline on the site, prints and returns the alerts.
///
/// # Panics
/// If the pipeline fails (a test bug).
#[must_use]
pub fn run_pipeline(site: &Site, truths: &[Vec<f64>], dt: f64, seed: u64) -> Vec<Alert> {
    let mut rng = Rng::new(seed);
    let recorded: Vec<(Vec<f64>, f64)> = truths
        .iter()
        .map(|truth| {
            let logger = Logger {
                rate: RATE,
                phase: rng.uniform() / RATE,
                noise: 0.02,
            };
            (sample(truth, dt, &logger, &mut rng), logger.phase)
        })
        .collect();
    let records: Vec<Record<'_>> = recorded
        .iter()
        .enumerate()
        .map(|(i, (samples, phase))| Record {
            sensor: i,
            series: Series {
                samples,
                start: Time::new(*phase),
                interval: Time::new(1.0 / RATE),
            },
        })
        .collect();
    let mut sink = CollectingSink::default();
    let alerts =
        process(site, &records, &PipelineConfig::default(), &mut sink).expect("pipeline runs");
    assert_eq!(sink.alerts, alerts, "every alert reached the sink");
    println!("{} alert(s):", alerts.len());
    for a in &alerts {
        println!("{}", a.render(&site.pipe_names));
    }
    alerts
}
