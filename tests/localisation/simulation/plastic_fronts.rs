//! Study V-S2, plastic pipes: with creeping (viscoelastic) walls, does the
//! quick front model - junction transmissions times exp(-kappa x) along
//! every plastic pipe - still match the full simulator?
//!
//! A plastic estate (uPVC-like: 110 mm, a = 400 m/s, illustrative wall
//! alpha D / e = 16.7, J = 1e-10 1/Pa, tau = 0.05 s - a front halves every
//! ~104 m [VERIFY real figures]), resting at 60 m:
//!
//! ```text
//!   R --300-- S1 --200-- T --300-- S2 (dead end)
//!                        |
//!                       350 (burst 150 m from T)
//!                        |
//!                        S3 (dead end)
//! ```
//!
//! An 8 mm hole opens suddenly. Each logger's first front is read at the
//! step it arrives (behind the front the creep brings part of the drop
//! back, so later reads are smaller drops).

use jk_hydraulics::boundary::leak;
use jk_hydraulics::pipe::viscoelastic::{KelvinVoigt, ViscoelasticWall};
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, NodeId, Pipe, Probe,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_localisation::Point;
use jk_localisation::placement::first_fronts;
use jk_tests::{check, check_close, pipe};

const D: f64 = 0.11;
const A: f64 = 400.0;

fn plastic(length: f64) -> Pipe {
    let wall = ViscoelasticWall::new(
        16.7,
        &[KelvinVoigt {
            compliance: 1e-10,
            retardation: Time::new(0.05),
        }],
    )
    .unwrap();
    pipe(length, D, A, 0.0).with_wall(wall)
}

/// The estate, with the burst branch split at the hole if `split`.
fn estate(split: bool) -> (Network, [NodeId; 3], Option<NodeId>) {
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
    net.add_pipe(r, s1, plastic(300.0)).unwrap();
    net.add_pipe(s1, t, plastic(200.0)).unwrap();
    net.add_pipe(t, s2, plastic(300.0)).unwrap();
    let hole = if split {
        let h = net.add_node(j());
        net.add_pipe(t, h, plastic(150.0)).unwrap();
        net.add_pipe(h, s3, plastic(200.0)).unwrap();
        Some(h)
    } else {
        net.add_pipe(t, s3, plastic(350.0)).unwrap();
        None
    };
    (net, [s1, s2, s3], hole)
}

#[test]
#[allow(clippy::too_many_lines)] // simulation, model and rigid comparison: one story
fn the_front_model_with_plastic_damping_matches_the_full_simulation() {
    let k = leak::coefficient(0.6, Area::of_circle(Length::new(0.008)), STANDARD_GRAVITY);
    let (mut net, sensors, hole) = estate(true);
    let hole = hole.expect("split");
    net.add_burst(Burst {
        node: hole,
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

    // The model on the unsplit map, the hole's flow found self-consistently.
    let b = A / (STANDARD_GRAVITY.value() * Area::of_circle(Length::new(D)).value());
    let mut h0: f64 = 0.0;
    for _ in 0..50 {
        h0 = 0.5 * b * k * (60.0 - h0).sqrt();
    }
    let flow = k * (60.0 - h0).sqrt();
    let (map, map_sensors, _) = estate(false);
    let model = first_fronts(
        &map,
        &map_sensors,
        Point {
            pipe: map.pipe_id(3).unwrap(),
            distance: Length::new(150.0),
        },
        Flow::new(flow),
    )
    .unwrap();
    let kappa = plastic(1.0)
        .wall()
        .unwrap()
        .front_damping(jk_kernel::WaveSpeed::new(A), STANDARD_GRAVITY);
    println!(
        "An 8 mm hole lets out {:.3} l/s, a {h0:.3} m drop at the hole; plastic walls keep \
         exp(-{kappa:.5} x) of a front",
        flow * 1000.0
    );
    for (i, name) in [
        "S1 (tee, 350 m)",
        "S2 (tee, 450 m, dead end)",
        "S3 (dead end, 200 m)",
    ]
    .iter()
    .enumerate()
    {
        let heads: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
        let start = (0.5 / dt) as usize;
        let at = (start..heads.len())
            .find(|&n| (heads[n] - 60.0).abs() > 1e-6)
            .expect("the front arrives");
        let felt = 60.0 - heads[at];
        check_close(
            &format!("first front at {name} [m]"),
            felt,
            model[i].value(),
            0.05,
        );
    }
    let rigid = first_fronts(
        &{
            let mut m = Network::new(STANDARD_GRAVITY);
            let j = || Node::Junction { demand: Flow::ZERO };
            let r = m.add_node(Node::Reservoir {
                head: Head::new(60.0),
            });
            let (s1, t, s2, s3) = (
                m.add_node(j()),
                m.add_node(j()),
                m.add_node(j()),
                m.add_node(j()),
            );
            for (x, y, l) in [
                (r, s1, 300.0),
                (s1, t, 200.0),
                (t, s2, 300.0),
                (t, s3, 350.0),
            ] {
                m.add_pipe(x, y, pipe(l, D, A, 0.0)).unwrap();
            }
            m
        },
        &map_sensors,
        Point {
            pipe: map.pipe_id(3).unwrap(),
            distance: Length::new(150.0),
        },
        Flow::new(flow),
    )
    .unwrap();
    println!(
        "  the same pipes without creep would give S1 {:.3} m, S2 {:.3} m, S3 {:.3} m",
        rigid[0].value(),
        rigid[1].value(),
        rigid[2].value()
    );
    check(
        "plastic walls cut the front at S2 to under a fifth",
        model[1].value() < 0.2 * rigid[1].value(),
    );
}
