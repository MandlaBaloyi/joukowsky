//! Low pressure in alerts (V-S7): the knee station of
//! `tests/common/knee.rs`, loggers at the pump (S1) and at the knee (S2),
//! the pump losing power (recorded) at t = 3 s. Loggers read gauge
//! pressure, so the knee's logger reads head - 50 m.
//!
//! With a light flywheel the column separates at the knee: the alert must
//! say so - and that the pressure went negative there (intrusion risk) -
//! whatever the event's class. With the usual flywheel it STILL separates
//! there (the knee sits too high for this main without surge protection),
//! and the alert says so too. Bursts on the estate, which never go below
//! zero, must not mention low pressure.

use jk_application::{Alert, Level, SensorSite, Site};
use jk_classification::Context;
use jk_hydraulics::{GridOptions, InitialCondition, Probe, SteadyOptions};
use jk_kernel::Time;
use jk_tests::check;
use jk_tests::knee::{KNEE, station};

use crate::run_pipeline;

/// Simulates the trip (water boils at -10.1 m) and runs the pipeline on
/// the two loggers' records.
fn alerts(inertia_share: f64, seed: u64) -> Vec<Alert> {
    let (net, pump, knee) = station(3.0, inertia_share, Some(-10.1), None);
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(12.0),
            &[jk_hydraulics::Probe::Node(pump), Probe::Node(knee)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 4,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let at_pump: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    let at_knee: Vec<f64> = run.trace.head(1).iter().map(|h| h.value() - KNEE).collect();
    let (site_net, pump, knee) = station(3.0, inertia_share, None, None);
    let site = Site {
        network: site_net,
        sensors: [(pump, "S1 pump"), (knee, "S2 knee")]
            .iter()
            .map(|&(node, name)| SensorSite {
                node,
                name: name.into(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
            })
            .collect(),
        pipe_names: vec!["suction".into(), "main up".into(), "main on".into()],
        context: Context::power_lost_at(&[Time::new(3.0)]),
        schedule: None,
    };
    run_pipeline(&site, &[at_pump, at_knee], dt, seed)
}

fn said(alerts: &[Alert], what: &str) -> bool {
    alerts
        .iter()
        .any(|a| a.reasons.iter().any(|r| r.contains(what)))
}

#[test]
fn vs7_column_separation_and_negative_pressure_are_reported() {
    println!("Pump trip, light flywheel (1/20): the column separates at the knee.");
    let light = alerts(0.05, 31);
    check(
        "  an alert says COLUMN SEPARATION at the knee",
        said(&light, "S2 knee: COLUMN SEPARATION"),
    );
    check(
        "  and NEGATIVE pressure there",
        said(&light, "S2 knee: NEGATIVE pressure"),
    );
    check(
        "  at Warning or above",
        light
            .iter()
            .filter(|a| a.reasons.iter().any(|r| r.contains("COLUMN SEPARATION")))
            .all(|a| a.level >= Level::Warning),
    );
    println!("\nThe same trip with the usual flywheel: the pump runs down slowly.");
    let heavy = alerts(1.0, 32);
    check(
        "  the column still separates at the knee - a flywheel alone does not protect this main",
        said(&heavy, "S2 knee: COLUMN SEPARATION"),
    );
    println!("\nControl: two bursts on the flowing estate (pressure stays well above 0):");
    let (truths, dt, site) = super::scenarios::flowing_two_bursts();
    let bursts = run_pipeline(&site, &truths, dt, 33);
    check(
        "  no alert mentions column separation or negative pressure",
        !said(&bursts, "COLUMN SEPARATION") && !said(&bursts, "NEGATIVE pressure"),
    );
}
