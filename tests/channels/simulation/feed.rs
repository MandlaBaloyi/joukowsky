//! Source to tap: the 5 km canal feeding a town through a 3 km DN600 main
//! (`crates/application/src/feed.rs`).
//!
//! ```text
//!   inflow -> canal (5 km) -> intake (outlet bed 100 m) -> 3 km DN600 -> town (300 l/s)
//! ```
//!
//! The canal gives the network the intake's level; the network gives the
//! canal the flow it draws; they take turns until they agree. Then: what
//! the town sees when the water into the canal stops.

use jk_application::feed::{CanalFeed, run_fed};
use jk_channels::{Initial, Upstream};
use jk_hydraulics::steady::extended::PeriodOptions;
use jk_hydraulics::{Network, Node, NodeId};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

use crate::canal;

const DEMAND: f64 = 0.3;
const BED: f64 = 100.0;
const HOUR: f64 = 3600.0;

fn town() -> (Network, NodeId, NodeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let intake = net.add_node(Node::Reservoir {
        head: Head::new(BED),
    });
    let town = net.add_node(Node::Junction {
        demand: Flow::new(DEMAND),
    });
    net.add_pipe(intake, town, pipe(3000.0, 0.6, 1000.0, 0.02))
        .unwrap();
    (net, intake, town)
}

fn feed(intake: NodeId, inflow: Vec<(Time, Flow)>) -> CanalFeed {
    CanalFeed {
        reach: canal(),
        upstream: Upstream::Inflow(inflow),
        initial: Initial::Uniform(Flow::new(DEMAND)),
        cells: 100,
        outlet_bed: Head::new(BED),
        intake,
        tolerance: Flow::new(1e-9),
    }
}

/// Main's friction loss at the demand [m]: R Q^2, R = f L / (2 g D A^2).
fn main_loss() -> f64 {
    let a = std::f64::consts::PI * 0.25 * 0.36;
    0.02 * 3000.0 / (2.0 * STANDARD_GRAVITY.value() * 0.6 * a * a) * DEMAND * DEMAND
}

#[test]
fn a_canal_and_the_town_it_feeds_agree_on_level_and_flow() {
    let (net, intake, town_node) = town();
    let period = PeriodOptions {
        duration: Time::new(6.0 * HOUR),
        ..PeriodOptions::default()
    };
    let run = run_fed(
        &net,
        &period,
        &feed(intake, vec![(Time::ZERO, Flow::new(DEMAND))]),
    )
    .unwrap();
    let hn = canal().normal_depth(Flow::new(DEMAND)).unwrap().value();
    println!(
        "300 l/s into the canal, 300 l/s drawn by the town: settled in {} turns (last change \
         {:.1e} m^3/s)",
        run.turns, run.change
    );
    let drawn_ok = run
        .drawn
        .iter()
        .all(|(_, q)| (q.value() - DEMAND).abs() < 1e-9);
    check(
        "  the town draws its 300 l/s at every recorded time",
        drawn_ok,
    );
    let outlet = run.canal.gauges[0].1.last().unwrap().value();
    check_close(
        "  the canal's outlet stays at normal depth [m]",
        outlet,
        hn,
        1e-6,
    );
    check_close(
        "  the town's head: bed + depth - the main's friction [m]",
        run.network.node_heads.last().unwrap()[town_node.index()].value(),
        BED + hn - main_loss(),
        1e-6,
    );
    check("  and the canal never ran dry", run.dry_from.is_none());
}

#[test]
fn when_the_canal_stops_flowing_the_town_feels_it_hours_later() {
    let (net, intake, town_node) = town();
    let period = PeriodOptions {
        duration: Time::new(10.0 * HOUR),
        ..PeriodOptions::default()
    };
    // The water into the canal stops between 1:00 and 1:10.
    let inflow = vec![
        (Time::ZERO, Flow::new(DEMAND)),
        (Time::new(HOUR), Flow::new(DEMAND)),
        (Time::new(HOUR + 600.0), Flow::ZERO),
    ];
    let run = run_fed(&net, &period, &feed(intake, inflow.clone())).unwrap();
    let c = &run.canal;
    let head = |k: usize| run.network.node_heads[k][town_node.index()].value();
    println!("The water into the canal stops at 01:00; the town keeps drawing 300 l/s:");
    for (k, t) in run.network.times.iter().enumerate().step_by(12) {
        println!("  {:5.1} h: town head {:7.3} m", t.value() / HOUR, head(k));
    }
    // Water drawn by the town: in all, and until the outlet first ran dry.
    let dt = period.step.value();
    let drawn_until = |until: f64| -> f64 {
        run.drawn
            .windows(2)
            .filter(|w| w[1].0.value() <= until + 1e-9)
            .map(|w| f64::midpoint(w[0].1.value(), w[1].1.value()) * dt)
            .sum()
    };
    println!(
        "  canal: held {:.0} m^3 at the start; {:.0} m^3 came in; {:.0} m^3 left it; {:.0} m^3 \
         remain",
        c.volume.0, c.exchanged.0, c.exchanged.1, c.volume.1
    );
    check_close(
        "  water in the canal: start + in - out = end [m^3]",
        c.volume.0 + c.exchanged.0 - c.exchanged.1,
        c.volume.1,
        1e-9,
    );
    let dry = run.dry_from.expect("the canal runs dry");
    let (asked, asked_till_dry) = (
        drawn_until(period.duration.value()),
        drawn_until(dry.value()),
    );
    println!(
        "  the outlet first runs dry at {:.2} h, {:.1} h after the cut - from then the town's \
         300 l/s cannot be met; water still trickles down from the upper canal",
        dry.value() / HOUR,
        dry.value() / HOUR - 1.0
    );
    check(
        &format!(
            "  the canal gave {:.0} m^3: all {asked_till_dry:.0} asked until it ran dry, never \
             more than the {asked:.0} asked in all",
            c.exchanged.1
        ),
        c.exchanged.1 >= asked_till_dry - 1.0 && c.exchanged.1 <= asked + 1e-6,
    );
    check(
        "  the town's head falls with the intake level after the cut",
        head(run.network.times.len() / 2) < head(0) - 0.1,
    );
}
