//! V-A8: storage tanks in an extended-period run (step R-2 of
//! `docs/02_physics_of_transients/sources_dams_rivers_canals.txt`).
//!
//! 1. A tank (plan area 100 m^2) drains through 1 km of 200 mm pipe
//!    (f = 0.02) into a reservoir at 20 m. The pipe carries
//!    Q = sqrt((h - 20) / R), R = f L / (2 g D A^2), so with u = sqrt(h - 20)
//!
//!    ```text
//!      A_t dh/dt = -Q    =>    u(t) = u0 - t / (2 A_t sqrt(R))
//!    ```
//!
//!    a straight line: the tank's level and the moment it reaches its floor
//!    are known exactly.
//! 2. A tank alone supplies a steady 5 l/s: its level falls in a straight
//!    line; at its floor it stops, and the run reports the supply lost -
//!    the warning the extended-period run exists to give.

use jk_hydraulics::steady::extended::{PeriodOptions, SupplyEvent};
use jk_hydraulics::{Network, Node, TankStorage};
use jk_kernel::{Area, Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const HOUR: f64 = 3600.0;

fn draining_tank() -> Network {
    let mut net = Network::new(STANDARD_GRAVITY);
    let tank = net.add_node(Node::Reservoir {
        head: Head::new(50.0),
    });
    let low = net.add_node(Node::Reservoir {
        head: Head::new(20.0),
    });
    net.add_pipe(tank, low, pipe(1000.0, 0.2, 1000.0, 0.02))
        .unwrap();
    net.add_tank(TankStorage {
        node: tank,
        area: Area::new(100.0),
        floor: Head::new(22.0),
        ceiling: Head::new(60.0),
    })
    .unwrap();
    net
}

/// Exact level of the draining tank at time t [s].
fn exact_level(t: f64) -> f64 {
    let g = STANDARD_GRAVITY.value();
    let a = std::f64::consts::PI * 0.25 * 0.2 * 0.2;
    let r = 0.02 * 1000.0 / (2.0 * g * 0.2 * a * a);
    let u = 30.0_f64.sqrt() - t / (2.0 * 100.0 * r.sqrt());
    20.0 + u * u
}

#[test]
fn va8_a_draining_tank_follows_the_exact_curve_and_empties_on_time() {
    let net = draining_tank();
    let run = |step: f64| {
        net.extended(&PeriodOptions {
            step: Time::new(step),
            duration: Time::new(24.0 * HOUR),
            ..PeriodOptions::default()
        })
        .unwrap()
    };
    let fine = run(300.0);
    let coarse = run(600.0);
    println!("Tank draining into a reservoir 30 m below, level vs the exact curve:");
    let mut errors = (0.0_f64, 0.0_f64);
    for hours in [2.0, 4.0, 8.0, 12.0] {
        let t = hours * HOUR;
        let at = |r: &jk_hydraulics::steady::extended::PeriodRun| {
            let k = r
                .times
                .iter()
                .position(|x| (x.value() - t).abs() < 1.0)
                .unwrap();
            r.tanks[0].1[k].value()
        };
        let exact = exact_level(t);
        check_close(
            &format!("  level at {hours:.0} h [m]"),
            at(&fine),
            exact,
            1e-4,
        );
        errors.0 = errors.0.max((at(&fine) - exact).abs());
        errors.1 = errors.1.max((at(&coarse) - exact).abs());
    }
    println!(
        "  largest error: {:.2e} m with 5-minute steps, {:.2e} m with 10-minute steps",
        errors.0, errors.1
    );
    check(
        "halving the step cuts the error at least threefold (second order)",
        errors.1 > 3.0 * errors.0,
    );

    // When it empties: u reaches sqrt(22 - 20).
    let g = STANDARD_GRAVITY.value();
    let a = std::f64::consts::PI * 0.25 * 0.2 * 0.2;
    let r = 0.02 * 1000.0 / (2.0 * g * 0.2 * a * a);
    let empties = (30.0_f64.sqrt() - 2.0_f64.sqrt()) * 2.0 * 100.0 * r.sqrt();
    let reported = fine.events.iter().find_map(|e| match e {
        SupplyEvent::TankEmpty { at, .. } => Some(at.value()),
        _ => None,
    });
    check_close(
        "  tank reaches its floor at [h]",
        reported.expect("the tank empties") / HOUR,
        empties / HOUR,
        1e-3,
    );
    check(
        "  and the reservoir below keeps the network supplied",
        !fine
            .events
            .iter()
            .any(|e| matches!(e, SupplyEvent::SupplyLost { .. })),
    );
}

#[test]
fn va8_a_tank_that_is_the_only_source_runs_dry_and_supply_is_lost() {
    let mut net = Network::new(STANDARD_GRAVITY);
    let tank = net.add_node(Node::Reservoir {
        head: Head::new(30.0),
    });
    let town = net.add_node(Node::Junction {
        demand: Flow::new(0.005),
    });
    net.add_pipe(tank, town, pipe(500.0, 0.15, 1000.0, 0.02))
        .unwrap();
    net.add_tank(TankStorage {
        node: tank,
        area: Area::new(50.0),
        floor: Head::new(25.0),
        ceiling: Head::new(35.0),
    })
    .unwrap();
    let run = net
        .extended(&PeriodOptions {
            duration: Time::new(24.0 * HOUR),
            ..PeriodOptions::default()
        })
        .unwrap();
    // 5 m x 50 m^2 = 250 m^3 above the floor, drawn at 5 l/s.
    let expected = 250.0 / 0.005;
    let empty = run.events.iter().find_map(|e| match e {
        SupplyEvent::TankEmpty { at, .. } => Some(at.value()),
        _ => None,
    });
    let lost = run.events.iter().find_map(|e| match e {
        SupplyEvent::SupplyLost { at, reason } => Some((at.value(), reason.clone())),
        _ => None,
    });
    println!("A tank alone supplying 5 l/s, 250 m^3 above its floor:");
    check_close(
        "  empties at [h]",
        empty.expect("it empties") / HOUR,
        expected / HOUR,
        1e-9,
    );
    let (at, reason) = lost.expect("supply is lost");
    println!("  supply lost at {:.2} h: {reason}", at / HOUR);
    check(
        "  supply is lost within one step of emptying",
        at - expected <= 300.0,
    );
}
