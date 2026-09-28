//! What the product actually says, in four situations.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2        (the estate, 60 m reservoir)
//!                        |
//!                       600
//!                        |
//!                        S3
//! ```
//!
//! A note on echoes: after an event the network rings. A first version of
//! the pipeline turned ONE burst into 12 alerts, including a second, false
//! alarm - the echoes re-triggered the detectors. Now each sensor is blind
//! for one echo round trip of the network, then listens through the
//! ringing at reduced sensitivity (a new event must stand out from the
//! ringing), and returns to full sensitivity once its record looks like
//! plain noise again. The first and third scenarios check the result.

use jk_application::Level;
use jk_classification::{Context, EventClass};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Burst, BurstOpening, InitialCondition, Network, Node, SteadyOptions};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::estate::{A, estate};
use jk_tests::{check, pipe};

use crate::{estate_site, run_pipeline, simulate};

/// A 10 mm hole opening instantly at `node` at `start` seconds.
fn hole(node: jk_hydraulics::NodeId, start: f64) -> Burst {
    Burst {
        node,
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(start),
        opening: BurstOpening::Instantaneous,
    }
}

fn at_rest() -> InitialCondition {
    InitialCondition::AtRest {
        head: Head::new(60.0),
    }
}

#[test]
fn one_burst_gives_exactly_one_alarm_on_the_right_pipe() {
    let (sim, b) = estate(Some(200.0));
    let mut net = sim.net;
    net.add_burst(hole(b.unwrap(), 3.0)).unwrap();
    let (truths, dt) = simulate(&net, &at_rest(), &sim.sensors, 15.0);
    println!("A 10 mm hole opens 200 m along the south branch at t = 3 s; 15 s recorded:");
    let alerts = run_pipeline(&estate_site(Context::unknown()), &truths, dt, 1);
    check("exactly one alert - no echo alerts", alerts.len() == 1);
    let a = &alerts[0];
    check("it is an ALARM", a.level == Level::Alarm);
    check("for a burst", a.class == EventClass::Burst);
    let loc = a.location.as_ref().expect("located");
    check(
        "on the south branch (pipe 3)",
        loc.most_likely.pipe.index() == 3,
    );
    check(
        "within 2 m of the truth",
        (loc.most_likely.distance.value() - 200.0).abs() < 2.0,
    );
    check("seen by all three sensors", a.sensors.len() == 3);
}

#[test]
fn a_quiet_network_raises_nothing() {
    let (sim, _) = estate(None);
    let (truths, dt) = simulate(&sim.net, &at_rest(), &sim.sensors, 15.0);
    println!("15 s of an undisturbed network, three noisy sensors:");
    let alerts = run_pipeline(&estate_site(Context::unknown()), &truths, dt, 2);
    check("no alerts", alerts.is_empty());
}

#[test]
fn two_bursts_far_apart_give_two_alarms() {
    // A FLOWING network (consumers draw 15 l/s at S2 and S3): friction then
    // damps the ringing between events, as it does in real pipes. (At rest
    // the simulator's quasi-steady friction barely damps at all - a known
    // model gap, docs 09 02 milestones S5/S6.)
    let main = |l| pipe(l, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = |q| Node::Junction {
        demand: Flow::new(q),
    };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(j(0.0));
    let tee = net.add_node(j(0.0));
    let s2 = net.add_node(j(0.015));
    let s3 = net.add_node(j(0.015));
    let b1 = net.add_node(j(0.0)); // 200 m along T-S3
    let b2 = net.add_node(j(0.0)); // 150 m along T-S2
    net.add_pipe(r, s1, main(400.0)).unwrap();
    net.add_pipe(s1, tee, main(300.0)).unwrap();
    net.add_pipe(tee, b2, main(150.0)).unwrap();
    net.add_pipe(b2, s2, main(350.0)).unwrap();
    net.add_pipe(tee, b1, main(200.0)).unwrap();
    net.add_pipe(b1, s3, main(400.0)).unwrap();
    net.add_burst(hole(b1, 3.0)).unwrap();
    net.add_burst(hole(b2, 30.0)).unwrap();
    let (truths, dt) = simulate(
        &net,
        &InitialCondition::Steady(SteadyOptions::default()),
        &[s1, s2, s3],
        40.0,
    );

    // The engine's map has the same consumers.
    let mut site = estate_site(Context::unknown());
    let (mut e, _) = estate(None);
    e.net = {
        let mut m = Network::new(STANDARD_GRAVITY);
        let r = m.add_node(Node::Reservoir {
            head: Head::new(60.0),
        });
        let s1 = m.add_node(j(0.0));
        let tee = m.add_node(j(0.0));
        let s2 = m.add_node(j(0.015));
        let s3 = m.add_node(j(0.015));
        m.add_pipe(r, s1, main(400.0)).unwrap();
        m.add_pipe(s1, tee, main(300.0)).unwrap();
        m.add_pipe(tee, s2, main(500.0)).unwrap();
        m.add_pipe(tee, s3, main(600.0)).unwrap();
        m
    };
    site.network = e.net;

    println!("Consumers drawing 30 l/s; a hole opens on the south branch at t = 3 s,");
    println!("another on the north branch at t = 30 s:");
    let alerts = run_pipeline(&site, &truths, dt, 3);
    let alarms: Vec<_> = alerts.iter().filter(|a| a.level == Level::Alarm).collect();
    check(
        "two alerts, both alarms - no echo alerts",
        alerts.len() == 2 && alarms.len() == 2,
    );
    if alarms.len() == 2 {
        let where_ = |i: usize| {
            alarms[i]
                .location
                .as_ref()
                .map(|l| (l.most_likely.pipe.index(), l.most_likely.distance.value()))
        };
        let (p1, d1) = where_(0).expect("located");
        let (p2, d2) = where_(1).expect("located");
        check(
            "the first on the south branch, ~200 m",
            p1 == 3 && (d1 - 200.0).abs() < 3.0,
        );
        check(
            "the second on the north branch, ~150 m",
            p2 == 2 && (d2 - 150.0).abs() < 3.0,
        );
    }
}

#[test]
fn a_faulty_sensor_is_sent_for_review_not_raised_as_an_alarm() {
    let (sim, _) = estate(None);
    let (mut truths, dt) = simulate(&sim.net, &at_rest(), &sim.sensors, 15.0);
    // S2's transducer glitches: its reading jumps 3 m at t = 6 s and stays
    // there. Nothing happened in the pipes.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let from = (6.0 / dt) as usize;
    for v in &mut truths[1][from..] {
        *v += 3.0;
    }
    println!("No event in the pipes; S2's reading jumps 3 m at t = 6 s (sensor fault):");
    let alerts = run_pipeline(&estate_site(Context::unknown()), &truths, dt, 4);
    check("one alert", alerts.len() == 1);
    let a = &alerts[0];
    check("sent for REVIEW, not an alarm", a.level == Level::Review);
    check("not located (one sensor only)", a.location.is_none());
    check(
        "names the sensor",
        a.sensors == vec!["S2 north block".to_string()],
    );
}
