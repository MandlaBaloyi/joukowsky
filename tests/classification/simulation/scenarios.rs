//! Eight scenarios: three kinds of event, sharp and slow, with and without
//! knowledge of the pumps' power supply.

use jk_classification::{Context, EventClass};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, Probe, Pump, PumpCurve,
    SteadyOptions, ValveClosure, thorley_inertia,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::{check, pipe};

use crate::{verdict, with_quiet_lead};

/// A burst on a main at rest (60 m), seen at a sensor 700 m away:
/// R --300-- S --700-- BURST --1300-- R. Returns (history, dt).
fn burst(opening: BurstOpening) -> (Vec<f64>, f64) {
    let main = |l| pipe(l, 0.15, 1000.0, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r1 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s = net.add_node(j());
    let b = net.add_node(j());
    let r2 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    net.add_pipe(r1, s, main(300.0)).unwrap();
    net.add_pipe(s, b, main(700.0)).unwrap();
    net.add_pipe(b, r2, main(1300.0)).unwrap();
    net.add_burst(Burst {
        node: b,
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(3.0),
        opening,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(8.0),
            &[Probe::Node(s)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 150,
            },
        )
        .unwrap();
    (
        run.trace.head(0).iter().map(|h| h.value()).collect(),
        run.trace.time_step().value(),
    )
}

/// A valve closing at the end of 1000 m of DN300 (100 m reservoir, valve
/// discharging to air), seen mid-pipe - upstream of the valve.
fn valve(closure: ValveClosure) -> (Vec<f64>, f64) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(100.0),
    });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure,
        flow_coefficient: Some(0.008),
    });
    let p = net.add_pipe(r, v, pipe(1000.0, 0.3, 1000.0, 0.02)).unwrap();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(6.0),
            &[Probe::Pipe {
                pipe: p,
                fraction: 0.5,
            }],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 200,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let h: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    (with_quiet_lead(&h, dt, 3.0), dt)
}

/// A pump station losing power: sump 10 m -> pump -> 2000 m rising main ->
/// 60 m reservoir, seen at the pump's discharge.
fn pump_trip() -> (Vec<f64>, f64) {
    let power = 998.2 * STANDARD_GRAVITY.value() * 0.10 * 65.0 / 0.8 / 1000.0;
    let station_pump = Pump {
        curve: PumpCurve::from_points(
            Head::new(80.0),
            (Flow::new(0.10), Head::new(65.0)),
            (Flow::new(0.15), Head::new(45.0)),
        )
        .unwrap(),
        rated_flow: Flow::new(0.10),
        rated_efficiency: 0.8,
        rated_speed_rpm: 1450.0,
        inertia: thorley_inertia(power, 1450.0),
    };
    let mut net = Network::new(STANDARD_GRAVITY);
    let sump = net.add_node(Node::Reservoir {
        head: Head::new(10.0),
    });
    let p = net.add_node(Node::Pump {
        pump: station_pump,
        trip: Some(Time::new(0.01)),
    });
    let top = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    net.add_pipe(sump, p, pipe(20.0, 0.3, 1000.0, 0.018))
        .unwrap();
    net.add_pipe(p, top, pipe(2000.0, 0.3, 1000.0, 0.018))
        .unwrap();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(6.0),
            &[Probe::Node(p)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 4,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let h: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    (with_quiet_lead(&h, dt, 3.0), dt)
}

/// When the pump trip's power loss happened on the logger's clock.
const PUMP_POWER_LOSS: f64 = 3.01;

#[test]
fn a_sharp_burst_is_called_a_burst() {
    println!("10 mm hole opening instantly, 700 m from the sensor, power not monitored:");
    let (h, dt) = burst(BurstOpening::Instantaneous);
    let c = verdict(&h, dt, &Context::unknown(), 1);
    check("burst", c.class == EventClass::Burst);
}

#[test]
fn an_instant_valve_closure_is_called_a_valve_closure() {
    println!("Valve shutting instantly, sensor mid-pipe upstream of it:");
    let (h, dt) = valve(ValveClosure::Instantaneous);
    let c = verdict(&h, dt, &Context::unknown(), 2);
    check("valve closure", c.class == EventClass::ValveClosure);
}

#[test]
fn a_slow_valve_closure_is_still_a_valve_closure() {
    println!("Valve shutting over 3 s, sensor mid-pipe upstream of it:");
    let (h, dt) = valve(ValveClosure::linear(Time::new(3.0)));
    let c = verdict(&h, dt, &Context::unknown(), 3);
    check("valve closure", c.class == EventClass::ValveClosure);
}

#[test]
fn a_pump_trip_is_recognised_from_its_shape_alone() {
    println!("Pump losing power, sensor at its discharge, power NOT monitored:");
    let (h, dt) = pump_trip();
    let c = verdict(&h, dt, &Context::unknown(), 4);
    check("pump trip", c.class == EventClass::PumpTrip);
}

#[test]
fn a_recorded_power_loss_makes_a_pump_trip_certain() {
    println!("Pump losing power, sensor at its discharge, power loss RECORDED:");
    let (h, dt) = pump_trip();
    let c = verdict(
        &h,
        dt,
        &Context::power_lost_at(&[Time::new(PUMP_POWER_LOSS)]),
        5,
    );
    check("pump trip", c.class == EventClass::PumpTrip);
    check("at least 99% confident", c.confidence >= 0.99);
}

#[test]
fn a_slow_burst_without_power_data_is_honestly_unknown() {
    println!("Hole opening over 0.5 s, power NOT monitored - shape alone cannot");
    println!("tell a slowly opening burst from a pump running down:");
    let (h, dt) = burst(BurstOpening::Linear {
        duration: Time::new(0.5),
    });
    let c = verdict(&h, dt, &Context::unknown(), 6);
    check("the classifier abstains", c.class == EventClass::Unknown);
    check(
        "and burst is among its top two candidates",
        c.probabilities
            .iter()
            .take(2)
            .any(|(k, _)| *k == EventClass::Burst),
    );
}

#[test]
fn power_data_settles_the_slow_burst() {
    println!("The same slow burst, power monitored and STABLE:");
    let (h, dt) = burst(BurstOpening::Linear {
        duration: Time::new(0.5),
    });
    let c = verdict(&h, dt, &Context::power_stable(), 7);
    check("burst", c.class == EventClass::Burst);
}

#[test]
fn a_sharp_burst_with_stable_power_is_a_confident_burst() {
    println!("Instant burst, power monitored and stable:");
    let (h, dt) = burst(BurstOpening::Instantaneous);
    let c = verdict(&h, dt, &Context::power_stable(), 8);
    check("burst", c.class == EventClass::Burst);
    check("at least 99% confident", c.confidence >= 0.99);
}
