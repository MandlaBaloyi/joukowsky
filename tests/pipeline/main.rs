//! Pipeline tests: sensor records in, alerts out.
//!
//! Code under test: `crates/application` (the event pipeline), driving every
//! domain context. Each scenario simulates events on the estate network,
//! records them with synthetic 128 S/s loggers, and checks the ALERTS - the
//! product's actual output.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only pipeline`

mod simulation;

use jk_application::{
    Alert, Chunk, CollectingSink, Monitor, PipelineConfig, Record, SensorSite, Site, process,
};
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
                range: None,
            })
            .collect(),
        pipe_names: vec![
            "R-S1 supply main".into(),
            "S1-T spine".into(),
            "T-S2 north branch".into(),
            "T-S3 south branch".into(),
        ],
        context,
        schedule: None,
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

/// One sensor's recording: samples from a 128 S/s logger.
pub struct Recorded {
    /// The samples (2 cm noise).
    pub samples: Vec<f64>,
    /// Time of the first sample (a random logger phase).
    pub start: f64,
}

/// Records each true head history with its own 128 S/s logger (random
/// phase, 2 cm noise).
#[must_use]
pub fn record(truths: &[Vec<f64>], dt: f64, seed: u64) -> Vec<Recorded> {
    let mut rng = Rng::new(seed);
    truths
        .iter()
        .map(|truth| {
            let logger = Logger {
                rate: RATE,
                phase: rng.uniform() / RATE,
                noise: 0.02,
            };
            Recorded {
                samples: sample(truth, dt, &logger, &mut rng),
                start: logger.phase,
            }
        })
        .collect()
}

/// Records the histories (see [`record`]), runs the pipeline over the
/// whole records, prints and returns the alerts.
///
/// # Panics
/// If the pipeline fails (a test bug).
#[must_use]
pub fn run_pipeline(site: &Site, truths: &[Vec<f64>], dt: f64, seed: u64) -> Vec<Alert> {
    run_pipeline_at(site, truths, dt, seed, 0.0)
}

/// As [`run_pipeline`], the records starting `utc` seconds after the UTC
/// epoch (so a site's schedule sees the hour of day).
///
/// # Panics
/// If the pipeline fails (a test bug).
#[must_use]
pub fn run_pipeline_at(
    site: &Site,
    truths: &[Vec<f64>],
    dt: f64,
    seed: u64,
    utc: f64,
) -> Vec<Alert> {
    let recorded = record(truths, dt, seed);
    let records: Vec<Record<'_>> = recorded
        .iter()
        .enumerate()
        .map(|(i, r)| Record {
            sensor: i,
            series: Series {
                samples: &r.samples,
                start: Time::new(utc + r.start),
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

/// An alert from a live feed, and how far the data had arrived when it
/// went out.
pub struct Issued {
    /// The alert.
    pub alert: Alert,
    /// Time of the latest sample received from any sensor when it went out.
    pub data_time: f64,
}

/// Feeds the recordings to a live monitor as a site would: every sensor
/// sends a chunk of `chunk` seconds in turn. Sensors in `delayed` send
/// nothing until the others have sent everything (a lost connection that
/// comes back); `cut` blanks one sensor's data between two times (a
/// gateway off during load-shedding: those samples never arrive). Prints
/// and returns the alerts in the order they went out.
///
/// # Panics
/// If the monitor fails (a test bug).
#[must_use]
pub fn live(
    site: &Site,
    recorded: &[Recorded],
    chunk: f64,
    delayed: &[usize],
    cut: Option<(usize, f64, f64)>,
) -> Vec<Issued> {
    let mut monitor = Monitor::new(site.clone(), PipelineConfig::default()).expect("monitor");
    let mut sink = CollectingSink::default();
    let mut issued = Vec::new();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let per = (chunk * RATE).round() as usize;
    let rounds = recorded
        .iter()
        .map(|r| r.samples.len().div_ceil(per))
        .max()
        .unwrap_or(0);
    let mut send = |monitor: &mut Monitor, sensor: usize, n: usize, issued: &mut Vec<Issued>| {
        let r = &recorded[sensor];
        let from = n * per;
        if from >= r.samples.len() {
            return;
        }
        let to = (from + per).min(r.samples.len());
        #[allow(clippy::cast_precision_loss)]
        let time = |k: usize| r.start + k as f64 / RATE;
        // Split around a power cut: the samples inside it never arrive.
        let mut pieces = vec![(from, to)];
        if let Some((s, a, b)) = cut
            && s == sensor
        {
            pieces = (from..to).filter(|&k| time(k) < a || time(k) >= b).fold(
                Vec::<(usize, usize)>::new(),
                |mut acc, k| {
                    match acc.last_mut() {
                        Some(last) if last.1 == k => last.1 = k + 1,
                        _ => acc.push((k, k + 1)),
                    }
                    acc
                },
            );
        }
        for (a, b) in pieces {
            let c = Chunk {
                start: Time::new(time(a)),
                interval: Time::new(1.0 / RATE),
                samples: &r.samples[a..b],
            };
            for alert in monitor
                .ingest(sensor, c, &mut sink)
                .expect("chunk accepted")
            {
                let data_time = monitor.lead().map_or(0.0, Time::value);
                issued.push(Issued { alert, data_time });
            }
        }
    };
    for n in 0..rounds {
        for s in (0..recorded.len()).filter(|s| !delayed.contains(s)) {
            send(&mut monitor, s, n, &mut issued);
        }
    }
    for &s in delayed {
        for n in 0..rounds {
            send(&mut monitor, s, n, &mut issued);
        }
    }
    let data_time = monitor.lead().map_or(0.0, Time::value);
    for alert in monitor.finish(&mut sink).expect("monitor finishes") {
        issued.push(Issued { alert, data_time });
    }
    assert_eq!(
        sink.alerts,
        issued.iter().map(|i| i.alert.clone()).collect::<Vec<_>>(),
        "every alert reached the sink"
    );
    println!("{} alert(s) went out:", issued.len());
    for i in &issued {
        println!(
            "{}
  -> sent when the data had reached t = {:.2} s",
            i.alert.render(&site.pipe_names),
            i.data_time
        );
    }
    issued
}
