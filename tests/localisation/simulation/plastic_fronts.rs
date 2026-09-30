//! Study V-S2, plastic pipes: with creeping (viscoelastic) walls, does the
//! quick model still match the full simulator - for the SHARP front (what
//! is timed) and for the drop FELT within a window after it (what decides
//! whether a logger notices the burst)?
//!
//! A plastic estate, resting at 60 m:
//!
//! ```text
//!   R --300-- S1 --200-- T --300-- S2 (dead end)
//!                        |
//!                       350 (burst 150 m from T)
//!                        |
//!                        S3 (dead end)
//! ```
//!
//! An 8 mm hole opens suddenly. Each logger's sharp front is read at the
//! step it arrives; the felt drop 0.25 s later (FN-023: behind the shrinking
//! front the creeping wall builds the drop back up).
//!
//! Two walls: an illustrative one (a = 400 m/s, alpha D / e = 16.7,
//! J = 1e-10 1/Pa, tau = 0.05 s - a front halves every ~104 m), and the HDPE
//! measured at Imperial College (Covas et al. 2005, five Kelvin-Voigt
//! elements - `reference::covas_hdpe`: a front halves every ~180 m).

use jk_hydraulics::boundary::leak;
use jk_hydraulics::pipe::viscoelastic::{KelvinVoigt, ViscoelasticWall, reference};
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, NodeId, Probe,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time, WaveSpeed};
use jk_localisation::Point;
use jk_localisation::placement::{felt_heads, first_fronts};
use jk_tests::{check, check_close, pipe};

/// A plastic pipe material: wall, wave speed, inner diameter.
#[derive(Clone, Copy)]
struct Material {
    wall: ViscoelasticWall,
    a: f64,
    d: f64,
}

fn illustrative() -> Material {
    let wall = ViscoelasticWall::new(
        16.7,
        &[KelvinVoigt {
            compliance: 1e-10,
            retardation: Time::new(0.05),
        }],
    )
    .unwrap();
    Material {
        wall,
        a: 400.0,
        d: 0.11,
    }
}

fn covas() -> Material {
    let (wall, a, d) = reference::covas_hdpe();
    Material {
        wall,
        a: a.value(),
        d: d.value(),
    }
}

/// The estate in `m` (rigid if `plastic` is false), with the burst branch
/// split at the hole if `split`.
fn estate(m: Material, plastic: bool, split: bool) -> (Network, [NodeId; 3], Option<NodeId>) {
    let make = |l: f64| {
        let p = pipe(l, m.d, m.a, 0.0);
        if plastic { p.with_wall(m.wall) } else { p }
    };
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let (s1, t, s2, s3) = (
        net.add_node(j()),
        net.add_node(j()),
        net.add_node(j()),
        net.add_node(j()),
    );
    net.add_pipe(r, s1, make(300.0)).unwrap();
    net.add_pipe(s1, t, make(200.0)).unwrap();
    net.add_pipe(t, s2, make(300.0)).unwrap();
    let hole = if split {
        let h = net.add_node(j());
        net.add_pipe(t, h, make(150.0)).unwrap();
        net.add_pipe(h, s3, make(200.0)).unwrap();
        Some(h)
    } else {
        net.add_pipe(t, s3, make(350.0)).unwrap();
        None
    };
    (net, [s1, s2, s3], hole)
}

/// Simulates the burst on the plastic estate and checks both readings
/// against the model; returns the modelled (sharp fronts, felt drops,
/// rigid-pipe fronts) at S1, S2, S3.
fn compare(m: Material, name: &str) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let k = leak::coefficient(0.6, Area::of_circle(Length::new(0.008)), STANDARD_GRAVITY);
    let (mut net, sensors, hole) = estate(m, true, true);
    net.add_burst(Burst {
        node: hole.expect("split"),
        coefficient: k,
        outside_head: Head::ZERO,
        start: Time::new(0.5),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(2.5),
            &sensors.map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 300,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();

    // The hole's flow, found self-consistently for the sharp drop there.
    let b = m.a / (STANDARD_GRAVITY.value() * Area::of_circle(Length::new(m.d)).value());
    let mut h0: f64 = 0.0;
    for _ in 0..50 {
        h0 = 0.5 * b * k * (60.0 - h0).sqrt();
    }
    let flow = Flow::new(k * (60.0 - h0).sqrt());
    let point = |net: &Network| Point {
        pipe: net.pipe_id(3).unwrap(),
        distance: Length::new(150.0),
    };
    let (map, map_sensors, _) = estate(m, true, false);
    let (rigid_map, _, _) = estate(m, false, false);
    let window = Time::new(0.25);
    let sharp = first_fronts(&map, &map_sensors, point(&map), flow).unwrap();
    let felt = felt_heads(&map, &map_sensors, point(&map), flow, window).unwrap();
    let rigid = first_fronts(&rigid_map, &map_sensors, point(&rigid_map), flow).unwrap();
    println!(
        "{name}: an 8 mm hole lets out {:.3} l/s; a sharp front keeps exp(-{:.5} x)",
        flow.value() * 1000.0,
        m.wall.front_damping(WaveSpeed::new(m.a), STANDARD_GRAVITY)
    );
    for (i, who) in [
        "S1 (tee, 350 m)",
        "S2 (tee, 450 m, dead end)",
        "S3 (dead end, 200 m)",
    ]
    .iter()
    .enumerate()
    {
        let heads: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
        let (start, later) = ((0.5 / dt) as usize, (window.value() / dt).round() as usize);
        let at = (start..heads.len())
            .find(|&n| (heads[n] - 60.0).abs() > 1e-6)
            .expect("the front arrives");
        println!("  {who}: rigid pipe would give {:.3} m", rigid[i].value());
        check_close(
            "    sharp front [m]",
            60.0 - heads[at],
            sharp[i].value(),
            0.05,
        );
        check_close(
            "    felt within 0.25 s [m]",
            60.0 - heads[at + later],
            felt[i].value(),
            0.10,
        );
    }
    let values = |h: &[Head]| [h[0].value(), h[1].value(), h[2].value()];
    (values(&sharp), values(&felt), values(&rigid))
}

#[test]
fn the_front_model_with_plastic_walls_matches_the_full_simulation() {
    let (sharp, felt, rigid) = compare(illustrative(), "Illustrative plastic wall");
    check(
        "the sharp front at S2 is under a fifth of rigid pipe's",
        sharp[1] < 0.2 * rigid[1],
    );
    check(
        "but within 0.25 s S2 feels over two thirds of it",
        felt[1] > 0.67 * rigid[1],
    );
}

#[test]
fn the_front_model_holds_for_measured_hdpe() {
    let (sharp, felt, rigid) = compare(covas(), "HDPE, Imperial College (Covas et al. 2005)");
    println!(
        "  at S2: sharp front {:.0}% of rigid pipe's, felt within 0.25 s {:.0}%",
        100.0 * sharp[1] / rigid[1],
        100.0 * felt[1] / rigid[1]
    );
    check("felt is more than the sharp front", felt[1] > sharp[1]);
}
