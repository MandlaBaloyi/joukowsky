//! V-A15: pressure-driven demand (EPANET 2.2's law,
//! `crates/domain/hydraulics/src/steady/gga.rs`).
//!
//! A reservoir at H feeds one consumer (ground z, full demand D) through a
//! pipe of resistance R. With `p_min` = 0, `p_req` = 15 m (span s) and the
//! exponent 0.5 the consumer gets
//!
//! ```text
//!   FULL     p >= 15 m:  Q = D
//!   PARTIAL  0 < p < 15: Q = D sqrt(p / s),  p = H - z - R Q^2
//!                        =>  Q^2 = D^2 (H - z) / (s + D^2 R)
//!   CUT      p <= 0:     Q = 0
//! ```

use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, PressureDemand, Probe, SteadyOptions,
};
use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const D: f64 = 0.02;
const SPAN: f64 = 15.0;

fn resistance() -> f64 {
    let a = std::f64::consts::PI * 0.25 * 0.15 * 0.15;
    0.02 * 1000.0 / (2.0 * STANDARD_GRAVITY.value() * 0.15 * a * a)
}

fn options(exponent: f64) -> SteadyOptions {
    SteadyOptions {
        pressure_demand: Some(PressureDemand {
            minimum: Head::ZERO,
            required: Head::new(SPAN),
            exponent,
        }),
        ..SteadyOptions::default()
    }
}

/// Reservoir at `h` -> 1 km DN150 -> a consumer on ground 30 m drawing
/// 20 l/s at full pressure.
fn one_consumer(h: f64) -> (Network, jk_hydraulics::NodeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir { head: Head::new(h) });
    let c = net.add_node(Node::Junction {
        demand: Flow::new(D),
    });
    net.set_elevation(c, Length::new(30.0)).unwrap();
    net.add_pipe(r, c, pipe(1000.0, 0.15, 1000.0, 0.02))
        .unwrap();
    (net, c)
}

#[test]
fn va15_a_consumer_gets_all_some_or_none_of_its_demand_as_its_pressure_allows() {
    let r = resistance();
    let z = 30.0;
    println!(
        "One consumer on ground 30 m, 20 l/s at full pressure (15 m); the main loses \
         R Q^2 = {:.1} m at 20 l/s:",
        r * D * D
    );
    // FULL: plenty of head.
    let (net, c) = one_consumer(80.0);
    let s = net.steady_state(&options(0.5)).unwrap();
    check_close(
        "  source at 80 m: all 20 l/s delivered [l/s]",
        s.delivered[c.index()].value() * 1000.0,
        20.0,
        1e-12,
    );
    check_close(
        "  ... at head 80 - R D^2 [m]",
        s.node_heads[c.index()].value(),
        80.0 - r * D * D,
        1e-12,
    );
    // PARTIAL: closed form.
    let h = 45.0;
    let (net, c) = one_consumer(h);
    let s = net.steady_state(&options(0.5)).unwrap();
    let q = D * ((h - z) / (SPAN + D * D * r)).sqrt();
    check_close(
        "  source at 45 m: Q = D sqrt((H - z) / (s + D^2 R)) [l/s]",
        s.delivered[c.index()].value() * 1000.0,
        q * 1000.0,
        1e-9,
    );
    let p = s.node_heads[c.index()].value() - z;
    check_close(
        "  ... and that is D sqrt(p / 15) at the pressure it gets [l/s]",
        s.delivered[c.index()].value() * 1000.0,
        D * (p / SPAN).sqrt() * 1000.0,
        1e-9,
    );
    check(
        "  ... asked 20 l/s, and the pipes carry what is delivered",
        (s.requested[c.index()].value() - D).abs() < 1e-15
            && s.max_continuity_residual.value() < 1e-12,
    );
    // CUT: the source below the consumer's ground.
    let (net, c) = one_consumer(25.0);
    let s = net.steady_state(&options(0.5)).unwrap();
    check(
        &format!(
            "  source at 25 m, below the ground: nothing delivered ({:.1e} l/s)",
            s.delivered[c.index()].value() * 1000.0
        ),
        s.delivered[c.index()].value() == 0.0,
    );
    // Another exponent: the law holds at the pressure found.
    let (net, c) = one_consumer(h);
    let s = net.steady_state(&options(0.75)).unwrap();
    let p = s.node_heads[c.index()].value() - z;
    check_close(
        "  exponent 0.75: delivered = D (p / 15)^0.75 [l/s]",
        s.delivered[c.index()].value() * 1000.0,
        D * (p / SPAN).powf(0.75) * 1000.0,
        1e-9,
    );
}

#[test]
fn va15_a_transient_from_a_pressure_starved_state_starts_still() {
    let (net, c) = one_consumer(45.0);
    let run = net
        .simulate(
            &InitialCondition::Steady(options(0.5)),
            Time::new(3.0),
            &[Probe::Node(c)],
            &GridOptions::default(),
        )
        .unwrap();
    let (lo, hi) = (run.trace.min_head(0).value(), run.trace.max_head(0).value());
    check(
        &format!(
            "Starting from the partly supplied state, 3 s with nothing happening: the \
             consumer's head stays put (spread {:.1e} m)",
            hi - lo
        ),
        hi - lo < 1e-6,
    );
}

#[test]
fn va15_a_draining_tank_warns_when_it_can_no_longer_meet_the_demand() {
    use jk_hydraulics::TankStorage;
    use jk_hydraulics::steady::extended::{PeriodOptions, SupplyEvent};
    use jk_kernel::Area;
    // A tank (50 m^2, at 30 m, floor 18 m) alone feeds a town on ground
    // 10 m drawing 5 l/s through 500 m of DN150. The supply counts as
    // short below 99% delivered: pressure 15 x 0.99^2 m at the town.
    let mut net = Network::new(STANDARD_GRAVITY);
    let tank = net.add_node(Node::Reservoir {
        head: Head::new(30.0),
    });
    let town = net.add_node(Node::Junction {
        demand: Flow::new(0.005),
    });
    net.set_elevation(town, Length::new(10.0)).unwrap();
    net.add_pipe(tank, town, pipe(500.0, 0.15, 1000.0, 0.02))
        .unwrap();
    net.add_tank(TankStorage {
        node: tank,
        area: Area::new(50.0),
        floor: Head::new(18.0),
        ceiling: Head::new(35.0),
    })
    .unwrap();
    let run = net
        .extended(&PeriodOptions {
            duration: Time::new(24.0 * 3600.0),
            steady: options(0.5),
            ..PeriodOptions::default()
        })
        .unwrap();
    let a = std::f64::consts::PI * 0.25 * 0.15 * 0.15;
    let r = 0.02 * 500.0 / (2.0 * STANDARD_GRAVITY.value() * 0.15 * a * a);
    let q99 = 0.99 * 0.005;
    let needed = 10.0 + SPAN * 0.99 * 0.99 + r * q99 * q99;
    // Above it the town gets 99-100%: the level falls at about 5 l/s.
    let expected = (30.0 - needed) * 50.0 / 0.005;
    let short = run.events.iter().find_map(|e| match e {
        SupplyEvent::SupplyShort { at, delivered } => Some((at.value(), *delivered)),
        _ => None,
    });
    println!(
        "A tank alone feeds a town on 10 m ground (5 l/s, full pressure 15 m); 99% delivery \
         needs the tank at {needed:.2} m or more:"
    );
    let (at, share) = short.expect("the supply runs short");
    println!(
        "  SUPPLY SHORT at {:.2} h ({:.0}% delivered at that record); expected from the \
         falling level {:.2} h",
        at / 3600.0,
        share * 100.0,
        expected / 3600.0
    );
    check(
        "  the warning comes at the first record after the level passes it",
        at >= expected - 30.0 && at - expected <= 300.0,
    );
    let last = run.supply.last().unwrap();
    println!(
        "  at the last record the town gets {:.2} of its {:.2} l/s",
        last.1.value() * 1000.0,
        last.0.value() * 1000.0
    );
    check(
        "  and it gets less and less, never more than asked",
        run.supply
            .iter()
            .all(|(asked, given)| given.value() <= asked.value() + 1e-15)
            && last.1.value() < 0.9 * last.0.value(),
    );
}
