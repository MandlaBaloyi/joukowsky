//! Study V-S2, devices (FN-029, gap G-19): does the quick front model
//! (placement/detectability.rs with graph/devices.rs) match the full
//! simulator when a burst's wave must cross a pressure-reducing valve or a
//! running pump - and are routes through a SHUT valve cut?
//!
//! ```text
//!   R (80 m) --300-- S1 --300-- PRV (50 m) --300-- S2 --200-- hole --200-- J (20 l/s)
//! ```
//!
//! Frictionless DN200 pipes, 1000 m/s. The valve drops 30 m at 20 l/s, so
//! for a small wave it is a resistance r = 2 x 30 / 0.02 = 3000 s/m^2
//! between pipes of impedance B = 3245 s/m^2: of the burst's front, S1 hears
//! T = 2B / (2B + r) = 68%. Then a second source holds the zone above
//! 50 m: the valve shuts, S1 hears nothing, and no route runs through it.
//! Last, a running pump: r = -dH/dQ, the droop of its curve.

use jk_hydraulics::ValveMode;
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, NodeId, Probe, Pump,
    PumpCurve, SteadyOptions, thorley_inertia,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_localisation::Point;
use jk_localisation::graph::TravelGraph;
use jk_localisation::placement::first_fronts;
use jk_tests::{check, check_close, pipe};

const Q0: f64 = 0.02;

/// Shut-off 80 m; 100 l/s at 65 m; run-out 150 l/s at 45 m (the pump of
/// tests/hydraulics/analytical/pumps).
fn station_pump() -> Pump {
    Pump {
        curve: PumpCurve::from_points(
            Head::new(80.0),
            (Flow::new(0.10), Head::new(65.0)),
            (Flow::new(0.15), Head::new(45.0)),
        )
        .unwrap(),
        rated_flow: Flow::new(0.10),
        rated_efficiency: 0.8,
        rated_speed_rpm: 1450.0,
        inertia: thorley_inertia(80.0, 1450.0),
    }
}

/// The zone; the burst pipe split at the hole if `split`; a second source
/// at 60 m feeding J if `second`. Returns the network, [S1, S2], the hole
/// node (split) and the index of the burst pipe (not split).
fn zone(split: bool, second: bool) -> (Network, [NodeId; 2], Option<NodeId>) {
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
        demand: Flow::new(Q0),
    });
    let p = |l: f64| pipe(l, 0.2, 1000.0, 0.0);
    net.add_pipe(r, s1, p(300.0)).unwrap();
    net.add_pipe(s1, v, p(300.0)).unwrap();
    net.add_pipe(v, s2, p(300.0)).unwrap();
    let hole = if split {
        let h = net.add_node(j0());
        net.add_pipe(s2, h, p(200.0)).unwrap();
        net.add_pipe(h, j, p(200.0)).unwrap();
        Some(h)
    } else {
        net.add_pipe(s2, j, p(400.0)).unwrap();
        None
    };
    if second {
        let b = net.add_node(Node::Reservoir {
            head: Head::new(60.0),
        });
        net.add_pipe(b, j, p(300.0)).unwrap();
    }
    (net, [s1, s2], hole)
}

/// The first front at each probe: the change at the first step it moves.
fn simulated_fronts(
    net: &Network,
    probes: &[NodeId],
    start: f64,
    seconds: f64,
    reaches: u32,
) -> Vec<f64> {
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(seconds),
            &probes.iter().map(|&n| Probe::Node(n)).collect::<Vec<_>>(),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: reaches,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    (0..probes.len())
        .map(|i| {
            let h: Vec<f64> = run.trace.head(i).iter().map(|x| x.value()).collect();
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
            let first = (start / dt) as usize;
            let base = h[first];
            (first..h.len())
                .find(|&n| (h[n] - base).abs() > 1e-6)
                .map_or(0.0, |n| base - h[n])
        })
        .collect()
}

/// An 8 mm hole, opening at 0.1 s at node `hole`; its flow for the sharp
/// drop at a head `head` in a pipe of impedance `b`.
fn burst(net: &mut Network, hole: NodeId, head: f64, b: f64) -> Flow {
    let k = leak::coefficient(0.6, Area::of_circle(Length::new(0.008)), STANDARD_GRAVITY);
    net.add_burst(Burst {
        node: hole,
        coefficient: k,
        outside_head: Head::ZERO,
        start: Time::new(0.1),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let mut h0: f64 = 0.0;
    for _ in 0..50 {
        h0 = 0.5 * b * k * (head - h0).sqrt();
    }
    Flow::new(k * (head - h0).sqrt())
}

#[test]
fn a_front_crossing_a_prv_is_heard_as_the_orifice_law_says() {
    let b = 1000.0 / (STANDARD_GRAVITY.value() * std::f64::consts::PI * 0.01);
    let (mut sim, sensors, hole) = zone(true, false);
    let flow = burst(&mut sim, hole.unwrap(), 50.0, b);
    let fronts = simulated_fronts(&sim, &sensors, 0.1, 1.5, 100);
    let (map, map_sensors, _) = zone(false, false);
    let point = Point {
        pipe: map.pipe_id(3).unwrap(),
        distance: Length::new(200.0),
    };
    let model = first_fronts(&map, &map_sensors, point, flow).unwrap();
    let r = 2.0 * 30.0 / Q0;
    println!(
        "An 8 mm hole below a PRV dropping 30 m at 20 l/s ({:.2} l/s out; r = {r:.0} s/m^2, \
         B = {b:.0} s/m^2):",
        flow.value() * 1000.0
    );
    check_close(
        "  S2, same side: front [m]",
        fronts[1],
        model[1].value(),
        0.02,
    );
    check_close(
        "  S1, across the valve: front [m]",
        fronts[0],
        model[0].value(),
        0.02,
    );
    check_close(
        "  share heard across the valve (simulated) vs 2B / (2B + r)",
        fronts[0] / fronts[1],
        2.0 * b / (2.0 * b + r),
        0.02,
    );
}

#[test]
fn a_shut_prv_passes_no_wave_and_no_route() {
    let b = 1000.0 / (STANDARD_GRAVITY.value() * std::f64::consts::PI * 0.01);
    let (mut sim, sensors, hole) = zone(true, true);
    let s = sim.steady_state(&SteadyOptions::default()).unwrap();
    println!(
        "A second source at 60 m holds the zone: the valve is {:?}",
        s.valves[0].1
    );
    let head = s.node_heads[hole.unwrap().index()].value();
    let flow = burst(&mut sim, hole.unwrap(), head, b);
    let fronts = simulated_fronts(&sim, &sensors, 0.1, 1.5, 100);
    let (map, map_sensors, _) = zone(false, true);
    let point = Point {
        pipe: map.pipe_id(3).unwrap(),
        distance: Length::new(200.0),
    };
    let model = first_fronts(&map, &map_sensors, point, flow).unwrap();
    check_close(
        "  S2, same side as the burst: simulated vs model front [m]",
        fronts[1],
        model[1].value(),
        0.02,
    );
    check(
        &format!(
            "  S1, beyond the shut valve: simulator {:.1e} m, model {:.1e} m - nothing",
            fronts[0],
            model[0].value()
        ),
        fronts[0].abs() < 1e-9 && model[0].value() == 0.0,
    );
    let times = TravelGraph::new(&map).times_from(map_sensors[0].index(), &{
        map.pipes()
            .iter()
            .map(|p| p.pipe.travel_time().value())
            .collect::<Vec<_>>()
    });
    check(
        "  and no route from S1 reaches the zone: the locator cannot place a burst there from S1",
        !times[map_sensors[1].index()].is_finite(),
    );
}

#[test]
fn a_front_crossing_a_running_pump_loses_what_its_curve_droops() {
    // Sump (10 m) --10-- S --10-- PUMP --1000-- hole --1000-- top (55 m).
    let build = |split: bool| {
        let mut net = Network::new(STANDARD_GRAVITY);
        let sump = net.add_node(Node::Reservoir {
            head: Head::new(10.0),
        });
        let s = net.add_node(Node::Junction { demand: Flow::ZERO });
        let pump = net.add_node(Node::Pump {
            pump: station_pump(),
            trip: None,
        });
        let top = net.add_node(Node::Reservoir {
            head: Head::new(55.0),
        });
        let d = |l: f64| pipe(l, 0.3, 1000.0, 0.0);
        net.add_pipe(sump, s, d(10.0)).unwrap();
        net.add_pipe(s, pump, d(10.0)).unwrap();
        let hole = if split {
            let h = net.add_node(Node::Junction { demand: Flow::ZERO });
            net.add_pipe(pump, h, d(1000.0)).unwrap();
            net.add_pipe(h, top, d(1000.0)).unwrap();
            Some(h)
        } else {
            net.add_pipe(pump, top, d(2000.0)).unwrap();
            None
        };
        (net, s, hole)
    };
    let (mut sim, s, hole) = build(true);
    let steady = sim.steady_state(&SteadyOptions::default()).unwrap();
    let q0 = steady.pipe_flows[2].value();
    let b = 1000.0 / (STANDARD_GRAVITY.value() * std::f64::consts::PI * 0.25 * 0.09);
    let head = steady.node_heads[hole.unwrap().index()].value();
    let flow = burst(&mut sim, hole.unwrap(), head, b);
    let fronts = simulated_fronts(&sim, &[s], 0.1, 1.2, 4);
    let (map, s_map, _) = build(false);
    let point = Point {
        pipe: map.pipe_id(2).unwrap(),
        distance: Length::new(1000.0),
    };
    let model = first_fronts(&map, &[s_map], point, flow).unwrap();
    let r = -station_pump().curve.slope(q0, 1.0);
    let h0 = 0.5 * b * flow.value();
    println!(
        "A burst 1 km up a rising main; the pump delivers {:.1} l/s, its curve droops \
         r = {r:.0} s/m^2 (B = {b:.0} s/m^2)",
        q0 * 1000.0
    );
    check_close(
        "  suction-side front (simulated) vs model [m]",
        fronts[0],
        model[0].value(),
        0.02,
    );
    check_close(
        "  share passed by the pump (simulated) vs 2B / (2B + r)",
        fronts[0] / h0,
        2.0 * b / (2.0 * b + r),
        0.02,
    );
}
