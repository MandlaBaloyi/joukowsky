//! Pumps and valves by the hour (gap G-19): a PRV that is shut at night.
//!
//! ```text
//!   R (80 m) --300-- S1 --300-- PRV (50 m) --300-- S2 --200-- hole --200-- J
//!                                                                          |
//!                                              T (55 m) --1000 m DN100-----+
//! ```
//!
//! J draws 15 l/s by day, 3 l/s from midnight to 06:00. At night the small
//! reservoir T alone holds the zone above 50 m, so the PRV is SHUT and no
//! pressure wave from the zone reaches S1. By day T cannot keep up, the
//! PRV is active and S1 hears the zone (partly - FN-029).
//!
//! A hole opens at 02:00 local time. Only S2 hears it. Without the site's
//! schedule the monitor takes the valve as at base (daytime) demand, open,
//! and reads S1's silence as evidence against S2: "a fault in S2". With
//! the schedule it knows the valve is shut at 02:00, so S1's silence means
//! nothing. At noon both loggers hear it and it is located - as far as two
//! loggers in a line can: beyond S2.

use jk_application::{Level, Schedule, Site};
use jk_classification::Context;
use jk_hydraulics::ValveMode;
use jk_hydraulics::steady::extended::PeriodOptions;
use jk_hydraulics::{InitialCondition, Network, Node, NodeId, SteadyOptions};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, pipe};

use super::scenarios::hole;
use crate::{run_pipeline_at, simulate};

const HOUR: f64 = 3600.0;
const DAY_DEMAND: f64 = 0.015;

/// The zone with J drawing `demand`; the burst pipe split at the hole if
/// `split`. Returns the network, [S1, S2] and the hole.
fn zone(demand: f64, split: bool) -> (Network, [NodeId; 2], Option<NodeId>) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let j0 = || Node::Junction { demand: Flow::ZERO };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let s1 = net.add_node(j0());
    let v = net.add_node(Node::ControlValve {
        mode: ValveMode::Prv {
            setting: Head::new(50.0),
        },
        open_resistance: 0.0,
        response: None,
    });
    let s2 = net.add_node(j0());
    let j = net.add_node(Node::Junction {
        demand: Flow::new(demand),
    });
    let t = net.add_node(Node::Reservoir {
        head: Head::new(55.0),
    });
    let p = |l: f64| pipe(l, 0.2, 1000.0, 0.02);
    net.add_pipe(r, s1, p(300.0)).unwrap();
    net.add_pipe(s1, v, p(300.0)).unwrap();
    net.add_pipe(v, s2, p(300.0)).unwrap();
    let h = if split {
        let h = net.add_node(j0());
        net.add_pipe(s2, h, p(200.0)).unwrap();
        net.add_pipe(h, j, p(200.0)).unwrap();
        Some(h)
    } else {
        net.add_pipe(s2, j, p(400.0)).unwrap();
        None
    };
    net.add_pipe(t, j, pipe(1000.0, 0.1, 1000.0, 0.02)).unwrap();
    (net, [s1, s2], h)
}

fn site(schedule: bool) -> Site {
    let (net, sensors, _) = zone(DAY_DEMAND, false);
    let mut pattern = vec![0.2; 6];
    pattern.extend([1.0; 18]);
    let names = ["S1 above the valve", "S2 in the zone"];
    Site {
        network: net,
        sensors: sensors
            .iter()
            .zip(names)
            .map(|(&node, name)| jk_application::SensorSite {
                node,
                name: name.into(),
                clock: None,
                uncalibrated_clock: Time::new(1e-6),
                range: None,
            })
            .collect(),
        pipe_names: ["R-S1", "S1-PRV", "PRV-S2", "S2-J", "T-J"]
            .iter()
            .map(|s| (*s).to_string())
            .collect(),
        context: Context::unknown(),
        schedule: schedule.then(|| Schedule {
            period: PeriodOptions {
                pattern,
                ..PeriodOptions::default()
            },
        }),
        utc_offset: Some(Time::new(2.0 * HOUR)),
    }
}

/// A 10 mm hole at 4 s into a 15 s record of the zone drawing `demand`.
fn burst_records(demand: f64) -> (Vec<Vec<f64>>, f64) {
    let (mut net, sensors, h) = zone(demand, true);
    let valve = net.steady_state(&SteadyOptions::default()).unwrap().valves[0].1;
    println!(
        "The zone drawing {:.0} l/s: the PRV is {valve:?}",
        demand * 1000.0
    );
    net.add_burst(hole(h.unwrap(), 4.0)).unwrap();
    simulate(
        &net,
        &InitialCondition::Steady(SteadyOptions::default()),
        &sensors,
        15.0,
    )
}

#[test]
fn a_night_burst_behind_a_shut_valve_is_not_blamed_on_the_one_logger_that_heard_it() {
    let (truths, dt) = burst_records(0.2 * DAY_DEMAND);
    // 00:00 UTC = 02:00 local (UTC+2).
    let says = |alerts: &[jk_application::Alert], what: &str| {
        alerts
            .iter()
            .any(|a| a.reasons.iter().any(|r| r.contains(what)))
    };
    println!("\n-- Without the schedule (valve taken as at base demand: open):");
    let blind = run_pipeline_at(&site(false), &truths, dt, 7, 0.0);
    check(
        "  S1's silence is read as evidence against S2",
        says(&blind, "S1 above the valve watched and felt nothing"),
    );
    println!("\n-- With the schedule:");
    let aware = run_pipeline_at(&site(true), &truths, dt, 7, 0.0);
    check("  one alert", aware.len() == 1);
    check(
        "  it says the valve is shut at 02:00",
        says(&aware, "at 02:00 (1 shut"),
    );
    check(
        "  and does not blame S2: S1 could not have heard it",
        !says(&aware, "felt nothing") && says(&aware, "no other sensor"),
    );
}

#[test]
fn at_noon_the_valve_is_active_and_both_loggers_locate_the_burst() {
    let (truths, dt) = burst_records(DAY_DEMAND);
    // 10:00 UTC = 12:00 local.
    let alerts = run_pipeline_at(&site(true), &truths, dt, 7, 10.0 * HOUR);
    check("  one alert", alerts.len() == 1);
    let a = &alerts[0];
    check("  heard by both loggers", a.sensors.len() == 2);
    check(
        "  it says the devices are as scheduled at 12:00, none shut",
        a.reasons
            .iter()
            .any(|r| r.contains("at 12:00") && !r.contains("shut")),
    );
    let loc = a.location.as_ref().expect("located");
    println!(
        "  located on {} at {:.1} m (truth: S2-J at 200 m); level {:?}",
        ["R-S1", "S1-PRV", "PRV-S2", "S2-J", "T-J"][loc.most_likely.pipe.index()],
        loc.most_likely.distance.value(),
        a.level
    );
    // Two loggers in a line fix only the time difference: every point
    // beyond S2 (away from S1) gives the same one - S2-J and T-J alike.
    // What they must get right: the burst is beyond S2, not between them
    // or behind S1, and the true pipe is in the answer.
    let share = |p: usize| {
        loc.pipes
            .iter()
            .filter(|x| x.pipe.index() == p)
            .map(|x| x.probability)
            .sum::<f64>()
    };
    check(
        &format!(
            "  beyond S2: {:.0}% on S2-J (the truth) and T-J together",
            100.0 * (share(3) + share(4))
        ),
        share(3) + share(4) > 0.99,
    );
    check("  the true pipe holds a real share", share(3) > 0.1);
    check("  and it is not dismissed", a.level != Level::Info);
}
