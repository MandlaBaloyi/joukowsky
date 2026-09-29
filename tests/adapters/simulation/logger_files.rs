//! Rung 3, end to end through FILES: a simulated burst, written by three
//! "loggers" as the CSV exports real loggers make, read back by the
//! logger reader, and fed to the live engine.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2
//!                        |
//!                       600   <- a 10 mm hole opens 200 m from T at 10:15:03
//!                        |
//!                        S3
//! ```
//!
//! Each file: two lines of metadata, UTC timestamps rounded to the
//! millisecond (so each logger's clock is off by up to half a millisecond,
//! which is realistic), pressure in bar to 4 decimals, 128 S/s, 2 cm of noise.
//! The engine must raise ONE Alarm on T-S3 within 2 m of 200 m, at the
//! right time of day. Nothing is downloaded: the files are written into
//! the build's scratch folder.

use std::fmt::Write as _;
use std::path::PathBuf;

use jk_application::{Chunk, CollectingSink, Level, Monitor, PipelineConfig, SensorSite, Site};
use jk_classification::Context;
use jk_files::logger::{CsvOptions, read};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Burst, BurstOpening, GridOptions, InitialCondition, Probe};
use jk_kernel::{Area, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::estate::estate;
use jk_tests::sensor::{Logger, Rng, sample};
use jk_tests::{check, check_close};

/// Metres of head in one bar (998.2 kg/m^3, g = 9.81).
const BAR: f64 = 1e5 / (998.2 * 9.81);
/// 2026-09-29 10:15:00 UTC, seconds since 1970 (day 20725).
const BASE: f64 = 20_725.0 * 86_400.0 + 10.0 * 3600.0 + 15.0 * 60.0;

/// A logger export: metadata, header, ms-rounded UTC stamps, bar.
fn export(name: &str, heads: &[f64], phase: f64) -> String {
    let mut s = format!("Logger,{name}\nRate,128 Hz\nTimestamp,Pressure (bar)\n");
    for (k, h) in heads.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let t = phase + k as f64 / 128.0; // seconds after 10:15:00 (< 60)
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let ms = (t * 1000.0).round() as u64;
        let _ = writeln!(
            s,
            "2026-09-29T10:15:{:02}.{:03}Z,{:.4}",
            ms / 1000,
            ms % 1000,
            h / BAR
        );
    }
    s
}

#[test]
#[allow(clippy::too_many_lines)] // files out, files in, engine: one story
fn a_burst_recorded_in_logger_files_raises_one_alarm_in_the_right_place() {
    // 1. The physics: 10 s, the hole opening at 3 s.
    let (sim, burst) = estate(Some(200.0));
    let mut net = sim.net;
    net.add_burst(Burst {
        node: burst.expect("split"),
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(3.0),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(10.0),
            &sim.sensors.map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();

    // 2. Three loggers write their exports.
    let names = ["S1 gatehouse", "S2 north block", "S3 south block"];
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("logger_files");
    std::fs::create_dir_all(&dir).unwrap();
    let mut rng = Rng::new(21);
    let mut paths = Vec::new();
    for (i, name) in names.iter().enumerate() {
        let truth: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
        let logger = Logger {
            rate: 128.0,
            phase: rng.uniform() / 128.0,
            noise: 0.02,
        };
        let heads = sample(&truth, dt, &logger, &mut rng);
        let path = dir.join(format!("logger_{}.csv", i + 1));
        std::fs::write(&path, export(name, &heads, logger.phase)).unwrap();
        paths.push(path);
    }
    println!("Wrote 3 logger exports to {}", dir.display());

    // 3. Read them back, as real files.
    let recordings: Vec<_> = paths
        .iter()
        .map(|p| read(p, &CsvOptions::default()).unwrap())
        .collect();
    for (r, name) in recordings.iter().zip(names) {
        println!(
            "  {name}: {} samples at {:.0} S/s from {}, {} run(s)",
            r.samples(),
            1.0 / r.interval.value(),
            r.first_stamp,
            r.runs.len()
        );
    }

    // 4. The live engine, fed 1 s chunks in turn.
    let (e, _) = estate(None);
    let site = Site {
        network: e.net,
        sensors: names
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
            "R-S1".into(),
            "S1-T".into(),
            "T-S2".into(),
            "T-S3 south branch".into(),
        ],
        context: Context::unknown(),
    };
    let mut monitor = Monitor::new(site, PipelineConfig::default()).unwrap();
    let mut sink = CollectingSink::default();
    let mut alerts = Vec::new();
    for n in 0..11 {
        for (sensor, r) in recordings.iter().enumerate() {
            let run = &r.runs[0];
            let from = n * 128;
            if from >= run.heads[0].len() {
                continue;
            }
            let to = (from + 128).min(run.heads[0].len());
            #[allow(clippy::cast_precision_loss)]
            let start = run.start.value() + from as f64 * r.interval.value();
            let chunk = Chunk {
                start: Time::new(start),
                interval: r.interval,
                samples: &run.heads[0][from..to],
            };
            alerts.extend(monitor.ingest(sensor, chunk, &mut sink).unwrap());
        }
    }
    alerts.extend(monitor.finish(&mut sink).unwrap());
    for a in &alerts {
        println!(
            "{}",
            a.render(&[
                "R-S1".into(),
                "S1-T".into(),
                "T-S2".into(),
                "T-S3 south branch".into()
            ])
        );
    }
    check("exactly one alert", alerts.len() == 1);
    let a = &alerts[0];
    check("an ALARM for a burst", a.level == Level::Alarm);
    let loc = a.location.as_ref().expect("located");
    check("on T-S3", loc.most_likely.pipe.index() == 3);
    check_close(
        "at 200 m (within 2 m: the ms-rounded clocks cost a little)",
        loc.most_likely.distance.value(),
        200.0,
        0.01,
    );
    check_close(
        "at 10:15:03 - seconds past 10:15:00",
        a.time.value() - BASE,
        3.0,
        0.002,
    );
}
