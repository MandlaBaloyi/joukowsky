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
use jk_kernel::{Head, Time};
use jk_tests::check;
use jk_tests::knee::{KNEE, station, station_rated};

use crate::{live, record, run_pipeline};

/// Simulates the trip (water boils at -10.1 m): the site, the two
/// loggers' true records (gauge), their time step and the true highest
/// pressure at the knee. The knee's logger has measuring `range` (it reads
/// nothing beyond it), if given.
pub fn scenario(inertia_share: f64, range: Option<(f64, f64)>) -> (Site, Vec<Vec<f64>>, f64, f64) {
    scenario_rated(inertia_share, range, None)
}

/// As [`scenario`], the site's pipes rated for `rating`.
fn scenario_rated(
    inertia_share: f64,
    range: Option<(f64, f64)>,
    rating: Option<Head>,
) -> (Site, Vec<Vec<f64>>, f64, f64) {
    let (net, pump, knee) = station(3.0, inertia_share, Some(-10.1), None);
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(40.0),
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
    let highest = at_knee.iter().copied().fold(f64::MIN, f64::max);
    let at_knee: Vec<f64> = match range {
        Some((lo, hi)) => at_knee.iter().map(|x| x.clamp(lo, hi)).collect(),
        None => at_knee,
    };
    let (site_net, pump, knee) = station_rated(3.0, inertia_share, None, None, rating);
    let site = Site {
        network: site_net,
        sensors: [(pump, "S1 pump"), (knee, "S2 knee")]
            .iter()
            .map(|&(node, name)| SensorSite {
                node,
                name: name.into(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
                range: (node == knee)
                    .then_some(range)
                    .flatten()
                    .map(|(lo, hi)| (Head::new(lo), Head::new(hi))),
            })
            .collect(),
        pipe_names: vec!["suction".into(), "main up".into(), "main on".into()],
        context: Context::power_lost_at(&[Time::new(3.0)]),
        schedule: None,
        utc_offset: None,
    };
    (site, vec![at_pump, at_knee], dt, highest)
}

/// The alerts from whole records, and the true highest knee pressure.
fn alerts(inertia_share: f64, range: Option<(f64, f64)>, seed: u64) -> (Vec<Alert>, f64) {
    let (site, truths, dt, highest) = scenario(inertia_share, range);
    (run_pipeline(&site, &truths, dt, seed), highest)
}

/// The collapse peak an alert reports at the knee, if any.
fn peak_said(alert: &Alert) -> Option<f64> {
    alert
        .reasons
        .iter()
        .find(|r| r.starts_with("S2 knee: in the"))
        .and_then(|r| r.split("peaked at ").nth(1))
        .and_then(|r| r.split(' ').next())
        .and_then(|v| v.parse::<f64>().ok())
}

fn said(alerts: &[Alert], what: &str) -> bool {
    alerts
        .iter()
        .any(|a| a.reasons.iter().any(|r| r.contains(what)))
}

#[test]
fn vs7_column_separation_and_negative_pressure_are_reported() {
    println!("Pump trip, light flywheel (1/20): the column separates at the knee.");
    let (light, truth_peak) = alerts(0.05, None, 31);
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
    // Whole records: the 30 s after the front are already in.
    let whole = light.iter().find_map(peak_said);
    jk_tests::check_close(
        "  it reports the collapse peak at the knee, against the true highest pressure [m]",
        whole.unwrap_or(f64::NAN),
        truth_peak,
        0.02,
    );
    println!("\nLive, the loggers sending 1 s chunks:");
    let (site, truths, dt, _) = scenario(0.05, None);
    let issued = live(&site, &record(&truths, dt, 31), 1.0, &[], None);
    let first = issued.first().map(|i| &i.alert);
    check(
        "  the first alert goes out within seconds, without the peak",
        first.is_some_and(|a| a.revision == 0 && peak_said(a).is_none()),
    );
    let revised = issued
        .iter()
        .find(|i| i.alert.revision > 0 && peak_said(&i.alert).is_some());
    check(
        &format!(
            "  and is REVISED with the peak once 30 s more are in (data time {:.1} s)",
            revised.map_or(f64::NAN, |i| i.data_time)
        ),
        revised.is_some_and(|i| i.data_time >= 33.0),
    );
    println!("\nThe same trip with the usual flywheel: the pump runs down slowly.");
    let (heavy, _) = alerts(1.0, None, 32);
    check(
        "  the column still separates at the knee - a flywheel alone does not protect this main",
        said(&heavy, "S2 knee: COLUMN SEPARATION"),
    );
    println!("\nThe light flywheel again, the knee's logger a 0-16 bar one (0 to 163 m):");
    let (clipped, _) = alerts(0.05, Some((0.0, 163.2)), 34);
    check(
        "  it says the reading held at the logger's minimum - negative pressure not ruled out",
        said(
            &clipped,
            "S2 knee: the reading held at the logger's minimum",
        ),
    );
    check(
        "  and claims no negative pressure or separation it could not see",
        !said(&clipped, "S2 knee: NEGATIVE") && !said(&clipped, "S2 knee: COLUMN SEPARATION"),
    );
    println!("\nControl: two bursts on the flowing estate (pressure stays well above 0):");
    let (truths, dt, site) = super::scenarios::flowing_two_bursts();
    let bursts = run_pipeline(&site, &truths, dt, 33);
    check(
        "  no alert mentions column separation or negative pressure",
        !said(&bursts, "COLUMN SEPARATION") && !said(&bursts, "NEGATIVE pressure"),
    );
}

#[test]
fn vs7_a_peak_above_the_pipes_rating_is_flagged() {
    use jk_hydraulics::pipe::pn;
    for (class, bar) in [("PN10", 10.0), ("PN16", 16.0)] {
        println!(
            "\nThe light-flywheel trip again, the main's pipes {class} ({:.1} m):",
            pn(bar).value()
        );
        let (site, truths, dt, highest) = scenario_rated(0.05, None, Some(pn(bar)));
        let alerts = run_pipeline(&site, &truths, dt, 35);
        let over = said(&alerts, "S2 knee: OVER RATING - the surge");
        if bar < 12.0 {
            check(
                "  the pump's own pipes run over PN10 every day (118 m): said as such, not blamed on the surge",
                said(&alerts, "S1 pump: OVER RATING IN NORMAL OPERATION"),
            );
        }
        println!("  true highest pressure at the knee: {highest:.1} m");
        check(
            &format!(
                "  {} at the knee",
                if highest > pn(bar).value() {
                    "flagged OVER RATING"
                } else {
                    "not flagged"
                }
            ),
            over == (highest > pn(bar).value()),
        );
    }
}
