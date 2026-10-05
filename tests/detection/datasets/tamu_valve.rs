//! D3 - a real valve closure in a real pipe network: the Texas A&M leak
//! testbed (Aghashahi, Sela, Banks, Data in Brief 48 (2023) 109148; CC BY
//! 4.0; data/README.md).
//!
//! 47 m of 152.4 mm PVC, looped (LO) and branched (BR), fed by a pump.
//! Two dynamic pressure sensors: P1 at the inlet, P2 at the far corner. In
//! the 20 "Transient" runs the outlet valve was shut by hand at about
//! second 20, so the flow fell from 0.47 l/s to 0 - the pressure must
//! RISE. Each run had one leak type (CC, GL, LC, OL) or none (NL).
//!
//! The files hold 25 600 readings a second (the paper says 51.2 kHz - FN-
//! 047). A real transient logger takes about 128 a second, each the
//! average over its interval, so the readings are averaged in blocks of
//! 200 - what such a logger would have recorded - and the engine's
//! detector, with its default settings, watches that.
//!
//! What the first run showed (FN-047): real noise WANDERS, and the
//! detector, learning only the jitter, saw 7-17 false fronts a run on the
//! looped rig and twice missed the closure. Fixed by learning the noise
//! about the trend and again every 10 s; now checked:
//!   - the far sensor P2 sees the closure as a RISE of at least 0.2 m
//!     between 16 and 23 s ("about second 20"), in every run;
//!   - before 16 s, while nothing happens, no sensor reports any front.
//!
//! The inlet sensor P1 is printed, not checked: on the looped rig its
//! recordings show almost nothing of the closure. Locating is NOT tested:
//! each file was started on its own (their lengths differ by up to 3.6 s),
//! so P1 and P2 share no clock.

use jk_detection::{Detector, DetectorConfig, Direction, Series};
use jk_files::logger::{CsvOptions, PressureUnit, read};
use jk_kernel::Time;
use jk_tests::{check, dataset};

/// Readings averaged into one logger sample (25 600 / 128).
const BLOCK: usize = 200;

/// The runs: topology, leak.
const RUNS: [&str; 10] = [
    "BR_NL", "BR_OL", "BR_CC", "BR_LC", "BR_GL", "LO_NL", "LO_OL", "LO_CC", "LO_LC", "LO_GL",
];

#[test]
#[ignore = "full suite: 20 real recordings of 900 000 readings each"]
fn d3_a_real_valve_closure_seen_by_128_per_second_loggers() {
    let options = CsvOptions {
        unit: Some(PressureUnit::Pascal),
        ..CsvOptions::default()
    };
    let detector = Detector::new(DetectorConfig::default()).unwrap();
    println!("  run     sensor  noise [m]  events: arrival [s] (step [m], rise/drop)");
    let (mut seen, mut early) = (0, Vec::new());
    for run in RUNS {
        for sensor in ["P1", "P2"] {
            let Some(path) = dataset(&format!(
                "../datasets/tamu-testbed/{run}_Transient_{sensor}.csv"
            )) else {
                return;
            };
            let rec = read(&path, &options).unwrap();
            let raw = &rec.runs[0].heads[0];
            let dt = rec.interval.value();
            let logger: Vec<f64> = raw
                .as_chunks::<BLOCK>()
                .0
                .iter()
                .map(|b| b.iter().sum::<f64>() / to_f64(BLOCK))
                .collect();
            let block = dt * to_f64(BLOCK);
            let series = Series {
                samples: &logger,
                start: Time::new(rec.runs[0].start.value() + block),
                interval: Time::new(block),
            };
            let d = detector.run(&series).unwrap();
            let closure = d.events.iter().any(|e| {
                e.direction == Direction::Rise
                    && e.step.value() >= 0.2
                    && (16.0..23.0).contains(&e.arrival.value())
            });
            if sensor == "P2" && closure {
                seen += 1;
            }
            early.extend(
                d.events
                    .iter()
                    .filter(|e| e.arrival.value() < 16.0)
                    .map(|e| format!("{run} {sensor} at {:.2} s", e.arrival.value())),
            );
            let events: Vec<String> = d
                .events
                .iter()
                .map(|e| {
                    format!(
                        "{:.3} ({:+.3}, {})",
                        e.arrival.value(),
                        e.step.value(),
                        if e.direction == Direction::Rise {
                            "rise"
                        } else {
                            "drop"
                        }
                    )
                })
                .collect();
            println!(
                "  {run}   {sensor}      {:.4}    {}",
                d.noise.value(),
                if events.is_empty() {
                    "none".to_string()
                } else {
                    events.join("  ")
                }
            );
        }
    }
    println!(
        "  the far sensor saw the closure in {seen} of {} runs",
        RUNS.len()
    );
    check(
        "P2 sees the valve closure as a rise near 20 s in every run",
        seen == RUNS.len(),
    );
    println!("  fronts reported before 16 s (nothing happens then): {early:?}");
    check(
        "no false front in the quiet first 16 s of any recording",
        early.is_empty(),
    );
}

#[allow(clippy::cast_precision_loss)] // small counts
fn to_f64(n: usize) -> f64 {
    n as f64
}
