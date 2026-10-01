//! Fatigue loading counted live (V-S8): a flat 2 km pump main
//! (`tests/common/knee.rs`, `flat_main`) whose pump trips at 3 s - the
//! pressure stays above atmospheric, so every swing is elastic water
//! hammer. Loggers at the pump's discharge (S1) and mid-main (S2), fed to
//! the monitor in 1 s chunks. Every swing is counted (rainflow, gated at
//! 0.5 m) into each sensor's loading; the monitor ranks the sensors by the
//! equivalent number of 50 m swings (S-N exponent m = 4 [VERIFY per
//! material]).
//!
//! Why not the knee station of `low_pressure.rs`: its rising main boils
//! along 70% of its length, and the cavity models' numerical pulses there
//! are counted as cycles whose number follows the grid (FN-038).

use jk_application::{Chunk, CollectingSink, Monitor, PipelineConfig, SensorSite, Site};
use jk_classification::Context;
use jk_health::{Spectrum, rainflow};
use jk_hydraulics::{GridOptions, InitialCondition, Probe, SteadyOptions};
use jk_kernel::Time;
use jk_tests::knee::{flat_main, flat_main_rated};
use jk_tests::{check, check_close};

use crate::{RATE, record};

/// The true pressure records at the pump and mid-main, for `duration` s,
/// on a grid with at least `reaches` reaches per pipe.
fn records(reaches: u32) -> (Vec<Vec<f64>>, f64) {
    let (net, pump, mid) = flat_main(3.0, 1.0);
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(40.0),
            &[Probe::Node(pump), Probe::Node(mid)],
            &GridOptions {
                max_wave_speed_adjustment: 0.05,
                min_reaches: reaches,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let heads = |j| run.trace.head(j).iter().map(|h| h.value()).collect();
    (vec![heads(0), heads(1)], dt)
}

#[test]
fn vs8_without_boiling_the_count_does_not_depend_on_the_grid() {
    println!("The flat main's pump trip on two grids (dt 5 ms and 20 ms):");
    let (fine, _) = records(4);
    let (coarse, _) = records(1);
    for (j, name) in ["S1 pump", "S2 mid-main"].iter().enumerate() {
        let n_eq = |x: &[f64]| jk_health::equivalent_cycles(&rainflow(x, 0.5).unwrap(), 50.0, 4.0);
        let lowest = fine[j].iter().copied().fold(f64::MAX, f64::min);
        println!("  {name}: lowest pressure {lowest:.1} m (never below atmospheric)");
        check_close(
            "    equivalent 50 m swings, coarse grid against fine",
            n_eq(&coarse[j]),
            n_eq(&fine[j]),
            0.05,
        );
    }
}

#[test]
fn vs8_the_monitor_counts_each_sensors_fatigue_loading_live() {
    let (truths, dt) = records(4);
    let (net, pump, mid) = flat_main(3.0, 1.0);
    let site = Site {
        network: net,
        sensors: [(pump, "S1 pump"), (mid, "S2 mid-main")]
            .iter()
            .map(|&(node, name)| SensorSite {
                node,
                name: name.into(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
                range: None,
            })
            .collect(),
        pipe_names: vec!["suction".into(), "main 1".into(), "main 2".into()],
        context: Context::power_lost_at(&[Time::new(3.0)]),
        schedule: None,
    };
    let recorded = record(&truths, dt, 41);
    let config = PipelineConfig::default();
    let mut monitor = Monitor::new(site.clone(), config).unwrap();
    let mut sink = CollectingSink::default();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let per = RATE as usize;
    let rounds = recorded
        .iter()
        .map(|r| r.samples.len().div_ceil(per))
        .max()
        .unwrap();
    for n in 0..rounds {
        for (s, r) in recorded.iter().enumerate() {
            let from = n * per;
            if from >= r.samples.len() {
                continue;
            }
            let to = (from + per).min(r.samples.len());
            #[allow(clippy::cast_precision_loss)]
            let start = r.start + from as f64 / RATE;
            monitor
                .ingest(
                    s,
                    Chunk {
                        start: Time::new(start),
                        interval: Time::new(1.0 / RATE),
                        samples: &r.samples[from..to],
                    },
                    &mut sink,
                )
                .unwrap();
        }
    }
    println!("The flat main's pump trip, 40 s of two loggers, counted live:");
    for ((sensor, spectrum), r) in site.sensors.iter().zip(monitor.loading()).zip(&recorded) {
        // The same record counted in one go, outside the monitor.
        let mut batch = Spectrum::new(config.fatigue_bin.value()).unwrap();
        batch.add(&rainflow(&r.samples, config.fatigue_gate.value()).unwrap());
        let biggest = spectrum.bins().map(|(r, _)| r).fold(0.0, f64::max);
        println!(
            "  {}: {:.1} cycles counted, the largest swing {biggest:.1} m",
            sensor.name,
            spectrum.total()
        );
        check_close(
            "    equivalent 50 m swings, live vs the whole record at once",
            spectrum.equivalent_cycles(50.0, 4.0),
            batch.equivalent_cycles(50.0, 4.0),
            1e-12,
        );
    }
    let ranking = monitor.loading_ranking(50.0, 4.0);
    println!("  ranked by equivalent 50 m swings (m = 4):");
    for (name, n) in &ranking {
        println!("    {name:<12} {n:8.2}");
    }
    check(
        "  the pump's discharge - where the down-surge starts, furthest from the reservoir that \
         holds the head - took the most",
        ranking[0].0 == "S1 pump",
    );
}

#[test]
fn vs7_a_peak_over_the_rating_late_in_the_ringing_comes_by_one_revision() {
    use jk_hydraulics::pipe::pn;
    let (truths, dt) = records(4);
    let truth_peak = truths[0].iter().copied().fold(f64::MIN, f64::max);
    let (net, pump, mid) = flat_main_rated(3.0, 1.0, Some(pn(10.0)));
    let site = Site {
        network: net,
        sensors: [(pump, "S1 pump"), (mid, "S2 mid-main")]
            .iter()
            .map(|&(node, name)| SensorSite {
                node,
                name: name.into(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
                range: None,
            })
            .collect(),
        pipe_names: vec!["suction".into(), "main 1".into(), "main 2".into()],
        context: Context::power_lost_at(&[Time::new(3.0)]),
        schedule: None,
    };
    println!(
        "The flat main's pump trip, its pipes PN10 ({:.1} m); the pressure at the pump peaks          at {truth_peak:.1} m, about 20 s after the front - long after the alert:",
        pn(10.0).value()
    );
    let issued = crate::live(&site, &record(&truths, dt, 42), 1.0, &[], None);
    let flagged = |a: &jk_application::Alert| {
        a.reasons
            .iter()
            .any(|r| r.starts_with("S1 pump: OVER RATING - the surge"))
    };
    check(
        "  the first alert goes out within seconds, without it",
        issued
            .first()
            .is_some_and(|i| !flagged(&i.alert) && i.data_time < 10.0),
    );
    check(
        "  ONE revision then adds it, once every logger's 30 s are in",
        issued.len() == 2 && issued[1].alert.revision == 1 && flagged(&issued[1].alert),
    );
}
