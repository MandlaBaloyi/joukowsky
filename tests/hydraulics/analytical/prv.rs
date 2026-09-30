//! V-A10: pressure-reducing valves (`crates/domain/hydraulics/src/
//! boundary/prv.rs`; physics in `docs/02_physics_of_transients/
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

use jk_hydraulics::boundary::prv::PrvState;
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
    let v = net.add_node(Node::PressureReducingValve {
        setting: Head::new(SETTING),
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

fn state_of(s: &jk_hydraulics::SteadySolution, v: NodeId) -> PrvState {
    s.prvs.iter().find(|(n, _)| *n == v).unwrap().1
}

#[test]
fn va10_a_prv_holds_its_setting_opens_when_it_cannot_and_shuts_against_backflow() {
    let q = 0.02;
    let (r1, r2) = (resistance(500.0, 0.2, 0.02), resistance(400.0, 0.2, 0.02));
    let opts = SteadyOptions::default();

    println!("ACTIVE: source at 80 m, setting {SETTING} m, 20 l/s drawn");
    let (net, v, j, _) = zone(80.0, q, None);
    let s = net.steady_state(&opts).unwrap();
    check("  the valve is active", state_of(&s, v) == PrvState::Active);
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
        state_of(&s, v) == PrvState::Open,
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
    check("  the valve is shut", state_of(&s, v) == PrvState::Closed);
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
    check("  the valve is active", state_of(&s, v) == PrvState::Active);
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
    let v = net.add_node(Node::PressureReducingValve {
        setting: Head::new(SETTING),
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
