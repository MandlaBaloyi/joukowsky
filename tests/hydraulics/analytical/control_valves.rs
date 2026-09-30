//! V-A10: pressure-reducing valves; V-A11: pressure-sustaining,
//! flow-control and throttle valves (`crates/domain/hydraulics/src/
//! boundary/control_valve.rs`; physics in `docs/02_physics_of_transients/
//! governing_equations_and_moc.txt`, section 6).
//!
//! A reservoir feeds a junction through pipe 1, the PRV and pipe 2. With a
//! fixed demand Q every head is known in closed form (R = f L / (2 g D A^2)
//! per pipe):
//!
//! ```text
//!   ACTIVE (source above the setting):  just downstream = setting,
//!                                        junction = setting - R2 Q^2
//!   OPEN   (source below the setting):  junction = source - (R1 + Rv + R2) Q^2
//!   CLOSED (a second source holds the zone above the setting): no flow
//! ```
//!
//! In a transient the valve is held at its steady opening, an orifice with
//! R = drop / Q0^2. A wave dH arriving from upstream raises the upstream
//! characteristic by 2 dH; the new flow solves
//! R Q^2 + (Bu + Bd) Q = R Q0^2 + (Bu + Bd) Q0 + 2 dH, and the wave passed
//! on is Bd (Q - Q0) - about 2 Bd dH / (Bu + Bd + 2 R Q0) for a small one.

use jk_hydraulics::ValveMode;
use jk_hydraulics::boundary::control_valve::ValveState;
use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, NodeId, PipeId, Probe, SourceLevel, SteadyOptions,
};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const SETTING: f64 = 50.0;

fn resistance(length: f64, diameter: f64, f: f64) -> f64 {
    let a = std::f64::consts::PI * 0.25 * diameter * diameter;
    f * length / (2.0 * STANDARD_GRAVITY.value() * diameter * a * a)
}

/// Open valve: K = 5 on a 200 mm valve.
fn open_resistance() -> f64 {
    let a = std::f64::consts::PI * 0.25 * 0.2 * 0.2;
    5.0 / (2.0 * STANDARD_GRAVITY.value() * a * a)
}

/// Source -> 500 m DN200 -> PRV -> 400 m DN200 -> junction drawing
/// `demand`; optionally a second source feeding the junction through
/// 300 m of DN150.
fn zone(source: f64, demand: f64, second: Option<f64>) -> (Network, NodeId, NodeId, [PipeId; 2]) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(source),
    });
    let v = net.add_node(Node::ControlValve {
        mode: ValveMode::Prv {
            setting: Head::new(SETTING),
        },
        open_resistance: open_resistance(),
    });
    let j = net.add_node(Node::Junction {
        demand: Flow::new(demand),
    });
    let p1 = net.add_pipe(r, v, pipe(500.0, 0.2, 1000.0, 0.02)).unwrap();
    let p2 = net.add_pipe(v, j, pipe(400.0, 0.2, 1000.0, 0.02)).unwrap();
    if let Some(h) = second {
        let b = net.add_node(Node::Reservoir { head: Head::new(h) });
        net.add_pipe(b, j, pipe(300.0, 0.15, 1000.0, 0.02)).unwrap();
    }
    (net, v, j, [p1, p2])
}

fn state_of(s: &jk_hydraulics::SteadySolution, v: NodeId) -> ValveState {
    s.valves.iter().find(|(n, _)| *n == v).unwrap().1
}

#[test]
fn va10_a_prv_holds_its_setting_opens_when_it_cannot_and_shuts_against_backflow() {
    let q = 0.02;
    let (r1, r2) = (resistance(500.0, 0.2, 0.02), resistance(400.0, 0.2, 0.02));
    let opts = SteadyOptions::default();

    println!("ACTIVE: source at 80 m, setting {SETTING} m, 20 l/s drawn");
    let (net, v, j, _) = zone(80.0, q, None);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  the valve is active",
        state_of(&s, v) == ValveState::Active,
    );
    check_close(
        "  junction head = setting - R2 Q^2 [m]",
        s.node_heads[j.index()].value(),
        SETTING - r2 * q * q,
        1e-9,
    );
    check_close(
        "  head just upstream = 80 - R1 Q^2 [m]",
        s.node_heads[v.index()].value(),
        80.0 - r1 * q * q,
        1e-9,
    );

    println!("OPEN: source at 45 m, below the setting");
    let (net, v, j, _) = zone(45.0, q, None);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  the valve is fully open",
        state_of(&s, v) == ValveState::Open,
    );
    check_close(
        "  junction head = 45 - (R1 + Rv + R2) Q^2 [m]",
        s.node_heads[j.index()].value(),
        45.0 - (r1 + open_resistance() + r2) * q * q,
        1e-9,
    );

    println!("CLOSED: a second source at 60 m holds the zone above the setting");
    let (net, v, j, [p1, p2]) = zone(80.0, 0.01, Some(60.0));
    let s = net.steady_state(&opts).unwrap();
    let r3 = resistance(300.0, 0.15, 0.02);
    check("  the valve is shut", state_of(&s, v) == ValveState::Closed);
    check(
        "  nothing passes it",
        s.pipe_flows[p1.index()].value() == 0.0 && s.pipe_flows[p2.index()].value() == 0.0,
    );
    check_close(
        "  junction fed from the second source alone: 60 - R3 Q^2 [m]",
        s.node_heads[j.index()].value(),
        60.0 - r3 * 0.01 * 0.01,
        1e-9,
    );

    println!("...until the zone draws 50 l/s: the second source alone would give");
    println!(
        "   {:.1} m, below the setting - the PRV opens and shares",
        60.0 - r3 * 0.05 * 0.05
    );
    let (net, v, j, [_, p2]) = zone(80.0, 0.05, Some(60.0));
    let s = net.steady_state(&opts).unwrap();
    check(
        "  the valve is active",
        state_of(&s, v) == ValveState::Active,
    );
    let q2 = s.pipe_flows[p2.index()].value();
    check_close(
        "  head just below it = setting [m] (junction + R2 q^2)",
        s.node_heads[j.index()].value() + r2 * q2 * q2,
        SETTING,
        1e-9,
    );
    println!(
        "  the PRV passes {:.1} l/s, the second source {:.1} l/s",
        q2 * 1000.0,
        (0.05 - q2) * 1000.0
    );
    check(
        "  continuity and energy hold (residuals)",
        s.max_continuity_residual.value() < 1e-9 && s.max_energy_residual.value() < 1e-9,
    );
}

#[test]
fn va10_a_wave_passes_a_prv_in_the_share_the_orifice_law_gives() {
    // Frictionless 1 km pipes of DN300 either side; the valve drops 30 m
    // at 50 l/s. The source rises 0.5 m in 10 ms at t = 0.1 s.
    let (q0, rise) = (0.05, 0.5);
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let v = net.add_node(Node::ControlValve {
        mode: ValveMode::Prv {
            setting: Head::new(SETTING),
        },
        open_resistance: 0.0,
    });
    let j = net.add_node(Node::Junction {
        demand: Flow::new(q0),
    });
    net.add_pipe(r, v, pipe(1000.0, 0.3, 1000.0, 0.0)).unwrap();
    let p2 = net.add_pipe(v, j, pipe(1000.0, 0.3, 1000.0, 0.0)).unwrap();
    net.add_level(SourceLevel {
        node: r,
        history: vec![
            (Time::ZERO, Head::new(80.0)),
            (Time::new(0.1), Head::new(80.0)),
            (Time::new(0.11), Head::new(80.0 + rise)),
        ],
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(3.0),
            &[Probe::Pipe {
                pipe: p2,
                fraction: 0.5,
            }],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 50,
            },
        )
        .unwrap();
    let heads = run.trace.head(0);
    let at = |t: f64| heads[run.trace.sample_at(Time::new(t))].value();
    // The front reaches the valve at 1.1 s and mid-pipe 2 at 1.6 s; the
    // echo from the junction is back at mid-pipe at 2.6 s.
    let before = at(1.0);
    let passed = at(2.2) - before;

    let b = 1000.0 / (STANDARD_GRAVITY.value() * std::f64::consts::PI * 0.25 * 0.3 * 0.3);
    let rv = (80.0 - SETTING) / (q0 * q0);
    // R Q^2 + 2B Q = R Q0^2 + 2B Q0 + 2 dH.
    let rhs = rv * q0 * q0 + 2.0 * b * q0 + 2.0 * rise;
    let q = (-2.0 * b + (4.0 * b * b + 4.0 * rv * rhs).sqrt()) / (2.0 * rv);
    let exact = b * (q - q0);
    let linear = 2.0 * b * rise / (2.0 * b + 2.0 * rv * q0);
    println!(
        "A 0.5 m wave meets a PRV dropping 30 m at 50 l/s (frozen orifice, 2 R Q0 = {:.0} s/m^2, \
         pipe impedance B = {b:.0} s/m^2):",
        2.0 * rv * q0
    );
    println!("  steady head below the valve {before:.4} m (= the setting)");
    check_close("  head below before the wave [m]", before, SETTING, 1e-6);
    check_close("  wave passed on [m] (orifice law)", passed, exact, 1e-4);
    println!(
        "  = {:.0}% of the wave (small-wave formula: {:.4} m, {:.0}%) - the rest is reflected",
        100.0 * passed / rise,
        linear,
        100.0 * linear / rise
    );
}

/// Source (80 m) -> 1000 m DN150 -> A (high zone, drawing `draw`) ->
/// 200 m DN150 -> valve -> 300 m DN150 -> low reservoir at `low`. Returns
/// the network, the valve, A, and the three pipes' resistances.
fn high_zone(mode: ValveMode, draw: f64, low: f64) -> (Network, NodeId, NodeId, [f64; 3]) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let a = net.add_node(Node::Junction {
        demand: Flow::new(draw),
    });
    let v = net.add_node(Node::ControlValve {
        mode,
        open_resistance: open_resistance(),
    });
    let l = net.add_node(Node::Reservoir {
        head: Head::new(low),
    });
    net.add_pipe(r, a, pipe(1000.0, 0.15, 1000.0, 0.02))
        .unwrap();
    net.add_pipe(a, v, pipe(200.0, 0.15, 1000.0, 0.02)).unwrap();
    net.add_pipe(v, l, pipe(300.0, 0.15, 1000.0, 0.02)).unwrap();
    let rs = [
        resistance(1000.0, 0.15, 0.02),
        resistance(200.0, 0.15, 0.02),
        resistance(300.0, 0.15, 0.02),
    ];
    (net, v, a, rs)
}

/// The root of a decreasing function on [lo, hi] (bisection).
fn root(f: impl Fn(f64) -> f64, mut lo: f64, mut hi: f64) -> f64 {
    for _ in 0..200 {
        let mid = f64::midpoint(lo, hi);
        if f(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    f64::midpoint(lo, hi)
}

#[test]
fn va11_a_psv_keeps_the_high_zone_up_and_opens_when_it_need_not() {
    let opts = SteadyOptions::default();
    let psv = ValveMode::Psv {
        setting: Head::new(70.0),
    };
    let draw = 0.005;
    println!("PSV set to 70 m between a high zone (5 l/s) and a low reservoir at 20 m:");
    let (net, v, a, [r1, r2, _]) = high_zone(psv, draw, 20.0);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  the valve is active",
        state_of(&s, v) == ValveState::Active,
    );
    check_close(
        "  head just above it = setting [m]",
        s.node_heads[v.index()].value(),
        70.0,
        1e-9,
    );
    // The flow q it lets on: 80 - R1 (draw + q)^2 - R2 q^2 = 70.
    let q = root(
        |q| 80.0 - r1 * (draw + q).powi(2) - r2 * q * q - 70.0,
        0.0,
        1.0,
    );
    check_close(
        "  what it lets through to the low reservoir [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        q * 1000.0,
        1e-8,
    );
    check_close(
        "  the high zone's head [m]",
        s.node_heads[a.index()].value(),
        80.0 - r1 * (draw + q).powi(2),
        1e-9,
    );

    println!("...and with the low reservoir at 72 m, above the setting: no need to throttle");
    let (net, v, _, [r1, r2, r3]) = high_zone(psv, draw, 72.0);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  the valve is fully open",
        state_of(&s, v) == ValveState::Open,
    );
    let rv = open_resistance();
    let q = root(
        |q| 80.0 - r1 * (draw + q).powi(2) - (r2 + rv + r3) * q * q - 72.0,
        0.0,
        1.0,
    );
    check_close(
        "  flow as through plain pipes [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        q * 1000.0,
        1e-8,
    );
}

#[test]
fn va11_an_fcv_passes_its_setting_and_no_more() {
    let opts = SteadyOptions::default();
    let (r1, r2, r3) = (
        resistance(1000.0, 0.15, 0.02),
        resistance(200.0, 0.15, 0.02),
        resistance(300.0, 0.15, 0.02),
    );
    let natural = root(
        |q| 80.0 - (r1 + r2 + open_resistance() + r3) * q * q - 20.0,
        0.0,
        1.0,
    );
    println!(
        "FCV between a source at 80 m and a reservoir at 20 m; fully open it would pass          {:.1} l/s",
        natural * 1000.0
    );
    let fcv = |l_s: f64| ValveMode::Fcv {
        flow: Flow::new(l_s / 1000.0),
    };
    let (net, v, a, _) = high_zone(fcv(10.0), 0.0, 20.0);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  set to 10 l/s: active",
        state_of(&s, v) == ValveState::Active,
    );
    check_close(
        "  passes exactly 10 l/s [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        10.0,
        1e-9,
    );
    check_close(
        "  head above it: 80 - (R1 + R2) q^2 [m]",
        s.node_heads[v.index()].value(),
        80.0 - (r1 + r2) * 0.01 * 0.01,
        1e-9,
    );
    let _ = a;
    let (net, v, _, _) = high_zone(fcv(100.0), 0.0, 20.0);
    let s = net.steady_state(&opts).unwrap();
    check(
        "  set to 100 l/s: fully open",
        state_of(&s, v) == ValveState::Open,
    );
    check_close(
        "  passes what it can [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        natural * 1000.0,
        1e-8,
    );
}

#[test]
fn va11_a_tcv_is_a_fixed_loss_both_ways() {
    let opts = SteadyOptions::default();
    let (r1, r2, r3) = (
        resistance(1000.0, 0.15, 0.02),
        resistance(200.0, 0.15, 0.02),
        resistance(300.0, 0.15, 0.02),
    );
    let rv = open_resistance();
    let q = ((80.0 - 20.0) / (r1 + r2 + rv + r3)).sqrt();
    println!("TCV (K = 5 on DN200) between reservoirs at 80 m and 20 m, then 20 m and 80 m:");
    let (net, _, _, _) = high_zone(ValveMode::Tcv, 0.0, 20.0);
    let s = net.steady_state(&opts).unwrap();
    check_close(
        "  forward flow sqrt(60 / sum R) [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        q * 1000.0,
        1e-8,
    );
    let mut back = Network::new(STANDARD_GRAVITY);
    let r = back.add_node(Node::Reservoir {
        head: Head::new(20.0),
    });
    let a = back.add_node(Node::Junction { demand: Flow::ZERO });
    let v = back.add_node(Node::ControlValve {
        mode: ValveMode::Tcv,
        open_resistance: rv,
    });
    let l = back.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    back.add_pipe(r, a, pipe(1000.0, 0.15, 1000.0, 0.02))
        .unwrap();
    back.add_pipe(a, v, pipe(200.0, 0.15, 1000.0, 0.02))
        .unwrap();
    back.add_pipe(v, l, pipe(300.0, 0.15, 1000.0, 0.02))
        .unwrap();
    let s = back.steady_state(&opts).unwrap();
    check_close(
        "  the same flow backwards [l/s]",
        s.pipe_flows[2].value() * 1000.0,
        -q * 1000.0,
        1e-8,
    );
}
