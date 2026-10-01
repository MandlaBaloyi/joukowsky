//! V-A17: column separation - a vapour cavity at a closed valve (DVCM,
//! `crates/domain/hydraulics/src/moc/network/cavity.rs`;
//! `docs/02_physics_of_transients/column_separation_and_intrusion.txt`
//! section 5).
//!
//! A frictionless 500 m DN200 pipe (a = 1000 m/s) from a 30 m reservoir
//! carries 2 m/s through a valve that shuts instantly. The valve end rises
//! by a V0 / g = 204 m; the wave comes back from the reservoir 2L/a = 1 s
//! later and, in linear theory, pulls the valve end down to 30 - 204 =
//! -174 m. Water cannot go below its vapour floor: a cavity opens there.
//!
//! The exact solution with the cavity (worked in the doc): the head at the
//! valve stays on the floor; each round trip T = 2L/a the column moving
//! away slows by 2 g dH / a (dH = reservoir head - piezometric floor), and
//! its average deceleration is that of a rigid column, g dH / L. So at
//! whole round trips n T after the cavity opens, its volume is EXACTLY the
//! rigid-column volume
//!
//! ```text
//!   V(t) = A (V0 t - g dH t^2 / (2 L)),   t = n T.
//! ```
//!
//! When the water comes back and fills it, the columns meet: the valve end
//! jumps to the reservoir head + (a/g) times the velocity arriving then.

use jk_hydraulics::vapour::{VAPOUR_PRESSURE_20C, vapour_floor};
use jk_hydraulics::{
    GridOptions, InitialCondition, Network, NetworkRun, Node, Probe, SourceLevel, SteadyOptions,
    ValveClosure, ValveMode,
};
use jk_kernel::{Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const L: f64 = 500.0;
const D: f64 = 0.2;
const A: f64 = 1000.0;
const H_R: f64 = 30.0;
const V0: f64 = 2.0;

/// Runs the closure for 12 s; the valve at `elevation` m, the vapour floor
/// `floor` (gauge, or none). Probes: the valve, mid-pipe.
fn closure(floor: Option<f64>, elevation: f64) -> NetworkRun {
    let area = std::f64::consts::PI * 0.25 * D * D;
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(H_R),
    });
    // Frictionless, so the valve passes V0 A under the full 30 m.
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Instantaneous,
        flow_coefficient: Some(V0 * area / H_R.sqrt()),
    });
    let p = net.add_pipe(r, v, pipe(L, D, A, 0.0)).unwrap();
    net.set_elevation(v, Length::new(elevation)).unwrap();
    net.set_vapour_floor(floor.map(Head::new)).unwrap();
    net.simulate(
        &InitialCondition::Steady(SteadyOptions::default()),
        Time::new(12.0),
        &[
            Probe::Node(v),
            Probe::Pipe {
                pipe: p,
                fraction: 0.5,
            },
        ],
        &GridOptions {
            max_wave_speed_adjustment: 0.001,
            min_reaches: 100,
        },
    )
    .unwrap()
}

/// Checks the cavity at the valve against the exact solution, the valve at
/// `elevation` and the vapour floor `gauge_floor`.
fn check_cavity(run: &NetworkRun, gauge_floor: f64, elevation: f64) {
    let floor_head = elevation + gauge_floor;
    let g = STANDARD_GRAVITY.value();
    let area = std::f64::consts::PI * 0.25 * D * D;
    let dt = run.trace.time_step().value();
    let heads: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    let mid: Vec<f64> = run.trace.head(1).iter().map(|h| h.value()).collect();
    let volume = &run.cavity_volumes[0];
    let at = |t: f64| {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let k = (t / dt).round() as usize;
        k
    };
    let round_trip = 2.0 * L / A;
    let dh = H_R - floor_head;
    let delta = g * dh / A; // velocity change per pass
    #[allow(clippy::cast_precision_loss)] // step counts are small
    let opened = volume.iter().position(|&v| v > 0.0).map(|k| k as f64 * dt);
    check_close(
        "  the cavity opens when the wave returns, 2L/a [s]",
        opened.unwrap_or(f64::NAN),
        round_trip,
        0.01,
    );
    let lowest = |h: &[f64]| h.iter().copied().fold(f64::INFINITY, f64::min);
    check_close(
        "  the lowest head at the valve is its vapour floor [m]",
        lowest(&heads),
        floor_head,
        1e-9,
    );
    // Mid-pipe stands at half the valve's elevation (the pipe is straight).
    check(
        &format!(
            "  mid-pipe never below its own floor ({:.2} m >= {:.2} m)",
            lowest(&mid),
            elevation / 2.0 + gauge_floor
        ),
        lowest(&mid) >= elevation / 2.0 + gauge_floor - 1e-9,
    );
    println!("  cavity volume at whole round trips after it opened:");
    let rigid = |t: f64| area * (V0 * t - g * dh * t * t / (2.0 * L));
    let mut n = 1.0;
    while rigid(n * round_trip) > 0.0 {
        let t = n * round_trip;
        check_close(
            &format!("    {n:.0} T: rigid column A (V0 t - g dH t^2 / 2L) [m^3]"),
            volume[at(round_trip + t)],
            rigid(t),
            0.01,
        );
        n += 1.0;
    }
    // Collapse: in the round trip k where the column has turned back and
    // the cavity empties, the valve end jumps to H_R + (a/g) v_in, the
    // velocity of the wave arriving then, v_in = -V0 + 2 k delta.
    // (Without friction it opens again on the next down-surge; the first
    // collapse is the one with a closed form.)
    let first = at(opened.unwrap_or(f64::NAN));
    #[allow(clippy::cast_precision_loss)]
    let closed = volume[first..]
        .iter()
        .position(|&v| v <= 0.0)
        .map_or(f64::NAN, |k| (first + k) as f64 * dt);
    let k = ((closed - round_trip) / round_trip).floor();
    let v_in = -V0 + 2.0 * k * delta;
    // ... until the next wave arrives, at the end of that round trip.
    let surge_at = at(closed);
    let next_wave = at(round_trip * (k + 2.0)).max(surge_at + 1);
    let peak = heads[surge_at..next_wave]
        .iter()
        .copied()
        .fold(f64::MIN, f64::max);
    println!(
        "  it collapses at t = {closed:.3} s (rigid column: {:.3} s)",
        round_trip + 2.0 * V0 * L / (g * dh)
    );
    check_close(
        "  collapse surge: H_R + (a/g) v_in [m]",
        peak,
        H_R + A * v_in / g,
        0.01,
    );
}

#[test]
fn va17_a_cavity_opens_at_the_vapour_floor_instead_of_minus_174_m() {
    let elastic = closure(None, 0.0);
    let lowest = elastic
        .trace
        .head(0)
        .iter()
        .map(|h| h.value())
        .fold(f64::INFINITY, f64::min);
    println!("Valve shut instantly on 2 m/s, 500 m from a 30 m reservoir.");
    println!("Linear theory (no vapour floor): the valve end falls to {lowest:.1} m.");
    check_close(
        "  = 30 - a V0 / g [m]",
        lowest,
        H_R - A * V0 / STANDARD_GRAVITY.value(),
        1e-3,
    );
    println!("With water's vapour floor at the coast (-10 m gauge):");
    let run = closure(Some(-10.0), 0.0);
    check_cavity(&run, -10.0, 0.0);
}

#[test]
fn va17_on_the_highveld_and_up_a_rise_the_cavity_is_bigger() {
    let g = STANDARD_GRAVITY;
    let floor = vapour_floor(Length::new(1700.0), VAPOUR_PRESSURE_20C, g).value();
    println!(
        "The same closure in Johannesburg (1700 m: vapour floor {floor:.2} m gauge), the valve \
         15 m above the reservoir's datum:"
    );
    let run = closure(Some(floor), 15.0);
    check_cavity(&run, floor, 15.0);
    let coast = closure(Some(-10.0), 0.0);
    let biggest = |r: &NetworkRun| r.cavity_volumes[0].iter().copied().fold(0.0, f64::max);
    println!(
        "  largest cavity {:.1} litres, against {:.1} litres at the coast on the flat",
        biggest(&run) * 1e3,
        biggest(&coast) * 1e3
    );
    check(
        "  bigger: less pressure to push the water back",
        biggest(&run) > biggest(&coast),
    );
}

/// A shut valve mid-line acts as a wall: 30 m reservoir -> 50 m -> shut
/// PRV -> 500 m -> reservoir B, all at rest at 30 m. B's level falls at
/// once to -8 m: the wave pulls the column toward B, and at the wall it
/// would reach 30 - 2 x 38 = -46 m. A cavity opens on the valve's
/// downstream side - the two-sided device logic. The same exact solution
/// holds with V1 = g 38 / a and dH = -8 - (-10) = 2 m.
#[test]
#[allow(clippy::too_many_lines)] // one scenario, built and checked in order
fn va17_a_column_pulled_away_from_a_shut_valve_leaves_a_cavity_behind_it() {
    let g = STANDARD_GRAVITY.value();
    let area = std::f64::consts::PI * 0.25 * D * D;
    let (drop_to, floor) = (-8.0, -10.0);
    let mut net = Network::new(STANDARD_GRAVITY);
    let a_node = net.add_node(Node::Reservoir {
        head: Head::new(H_R),
    });
    let wall = net.add_node(Node::ControlValve {
        mode: ValveMode::Prv {
            setting: Head::ZERO,
        },
        open_resistance: 1.0,
        response: None,
    });
    let b_node = net.add_node(Node::Reservoir {
        head: Head::new(H_R),
    });
    net.add_pipe(a_node, wall, pipe(50.0, D, A, 0.0)).unwrap();
    let p = net.add_pipe(wall, b_node, pipe(L, D, A, 0.0)).unwrap();
    net.add_level(SourceLevel {
        node: b_node,
        history: vec![
            (Time::ZERO, Head::new(H_R)),
            (Time::new(1e-6), Head::new(drop_to)),
        ],
    })
    .unwrap();
    net.set_vapour_floor(Some(Head::new(floor))).unwrap();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(25.0),
            &[
                Probe::Pipe {
                    pipe: p,
                    fraction: 0.0,
                },
                Probe::Node(wall),
            ],
            &GridOptions {
                max_wave_speed_adjustment: 0.001,
                min_reaches: 10,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let below: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    let above: Vec<f64> = run.trace.head(1).iter().map(|h| h.value()).collect();
    let volume = &run.cavity_volumes[0];
    let lowest = |h: &[f64]| h.iter().copied().fold(f64::INFINITY, f64::min);
    println!("Reservoir B drops from 30 m to {drop_to} m behind a shut valve, 500 m away:");
    check_close(
        "  the valve's downstream side holds at the vapour floor [m]",
        lowest(&below),
        floor,
        1e-9,
    );
    check_close(
        "  its upstream side stays at rest at 30 m [m]",
        lowest(&above),
        H_R,
        1e-9,
    );
    let first = volume
        .iter()
        .position(|&v| v > 0.0)
        .expect("a cavity opens");
    #[allow(clippy::cast_precision_loss)]
    let origin = (first - 1) as f64 * dt;
    check_close(
        "  it opens when the wave arrives, L/a after B fell [s]",
        origin,
        L / A,
        1e-6,
    );
    let (v1, dh, round_trip) = (g * (H_R - drop_to) / A, drop_to - floor, 2.0 * L / A);
    let rigid = |t: f64| area * (v1 * t - g * dh * t * t / (2.0 * L));
    let at = |t: f64| {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let k = (t / dt).round() as usize;
        k
    };
    let peak = volume.iter().copied().fold(0.0, f64::max);
    let mut worst: f64 = 0.0;
    let mut n = 1.0;
    while rigid(n * round_trip) > 0.0 {
        let t = n * round_trip;
        worst = worst.max((volume[at(origin + t)] - rigid(t)).abs() / peak);
        n += 1.0;
    }
    // Its first life (without friction the column keeps bouncing).
    #[allow(clippy::cast_precision_loss)]
    let life = volume[first..].iter().take_while(|&&v| v > 0.0).count() as f64 * dt;
    println!(
        "  the cavity grows to {:.1} litres (rigid column: {:.1}) and lives {life:.2} s (rigid column: {:.2} s)",
        peak * 1e3,
        area * v1 * v1 * L / (2.0 * g * dh) * 1e3,
        2.0 * v1 * L / (g * dh)
    );
    check(
        &format!(
            "  at all {:.0} whole round trips its volume is the rigid column's (worst {worst:.1e} of the peak)",
            n - 1.0
        ),
        worst < 1e-9,
    );
}
