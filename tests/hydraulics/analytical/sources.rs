//! V-A7: sources whose level changes with time (R-1 of
//! `docs/02_physics_of_transients/sources_dams_rivers_canals.txt`).
//!
//! Two reservoirs joined by 1 km of 300 mm pipe, no friction, water at
//! rest at 50 m. The upstream reservoir's level then rises steadily, 1 cm a
//! second (a filling dam, a pump starting to fill a tower). A slow change
//! moves the whole water column together - rigid-column theory:
//!
//! ```text
//!   (L / (g A)) dQ/dt = H1(t) - H2 = r t    =>    Q(t) = g A r t^2 / (2 L)
//! ```
//!
//! The simulator knows nothing of this: it only follows the level history at
//! the reservoir node, with waves running up and down the pipe. Its flow
//! must follow the rigid-column answer, the waves only rippling around it -
//! and exactly on it at whole multiples of the pipe's period, 2L/a.

use jk_hydraulics::{GridOptions, InitialCondition, Network, Node, Probe, SourceLevel};
use jk_kernel::{Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const L: f64 = 1000.0;
const D: f64 = 0.3;
const RATE: f64 = 0.01;

fn two_reservoirs(history: Option<Vec<(Time, Head)>>) -> (Network, jk_hydraulics::PipeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let up = net.add_node(Node::Reservoir {
        head: Head::new(50.0),
    });
    let down = net.add_node(Node::Reservoir {
        head: Head::new(50.0),
    });
    let p = net.add_pipe(up, down, pipe(L, D, 1000.0, 0.0)).unwrap();
    if let Some(history) = history {
        net.add_level(SourceLevel { node: up, history }).unwrap();
    }
    (net, p)
}

fn flows(net: &Network, p: jk_hydraulics::PipeId, seconds: f64) -> (Vec<f64>, f64) {
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(50.0),
            },
            Time::new(seconds),
            &[Probe::Pipe {
                pipe: p,
                fraction: 0.5,
            }],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 20,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    (run.trace.flow(0).iter().map(|q| q.value()).collect(), dt)
}

#[test]
fn va7_a_slowly_rising_source_drives_the_water_column_as_theory_says() {
    let seconds = 60.0;
    let history = vec![
        (Time::ZERO, Head::new(50.0)),
        (Time::new(seconds), Head::new(50.0 + RATE * seconds)),
    ];
    let (net, p) = two_reservoirs(Some(history));
    let (q, dt) = flows(&net, p, seconds);
    let area = std::f64::consts::PI * 0.25 * D * D;
    let g = STANDARD_GRAVITY.value();
    println!("Upstream reservoir rising {RATE} m/s; flow at mid-pipe vs rigid-column theory:");
    // Whole multiples of the pipe's period 2L/a = 2 s, where the ripples of
    // the waves cancel exactly for a steady ramp, and times in between,
    // where they show.
    for t in [20.0, 21.5, 40.0, 40.7, 60.0] {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
        let k = (t / dt).round() as usize;
        let theory = g * area * RATE * t * t / (2.0 * L);
        check_close(
            &format!("  flow at {t:.1} s [l/s]"),
            q[k] * 1000.0,
            theory * 1000.0,
            0.02,
        );
    }
}

#[test]
fn a_flat_level_history_is_exactly_a_fixed_reservoir() {
    let flat = vec![
        (Time::ZERO, Head::new(50.0)),
        (Time::new(10.0), Head::new(50.0)),
    ];
    let (with, p) = two_reservoirs(Some(flat));
    let (without, _) = two_reservoirs(None);
    let (a, _) = flows(&with, p, 10.0);
    let (b, _) = flows(&without, p, 10.0);
    check("identical flows, step for step", a == b);
}
