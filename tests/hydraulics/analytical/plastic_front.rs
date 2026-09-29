//! V-A6: in a plastic pipe a sharp wave front shrinks by a fixed factor per
//! metre - the rate theory gives (milestone S6, gap G-02).
//!
//! For a creeping wall with Kelvin-Voigt elements (`J_k`, `tau_k`), the
//! wave number of a component of frequency omega is
//!
//! ```text
//!   gamma = (i omega / a) sqrt(1 + sum_k K_k / (1 + i omega tau_k)),
//!   K_k = 2 a^2 c J_k / g,   c = rho g alpha D / (2 e)
//! ```
//!
//! A SHARP front is made of the highest frequencies (omega tau >> 1), for
//! which sqrt(...) ~ 1 + sum `K_k` / (2 i omega `tau_k`), so
//!
//! ```text
//!   gamma ~ i omega / a + sum_k K_k / (2 a tau_k)
//! ```
//!
//! - every frequency in the front travels at a and is damped at the SAME
//!   rate kappa = `sum_k` a c `J_k` / (g `tau_k`) per metre: the front keeps its
//!   shape and shrinks as exp(-kappa x). Behind it, the slower creep brings
//!   part of the drop back over a few tau.
//!
//! Test: a long plastic pipe, a valve at one end slammed shut; the jump of
//! the first front is measured at points along the pipe (before any
//! reflection returns) and compared with exp(-kappa x).

use jk_hydraulics::pipe::viscoelastic::{KelvinVoigt, ViscoelasticWall};
use jk_hydraulics::{GridOptions, InitialCondition, Network, Node, Probe, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const L: f64 = 2000.0;
const A: f64 = 400.0;
const D: f64 = 0.1;
const V: f64 = 0.2;
const ALPHA_D_OVER_E: f64 = 16.7;
const J: f64 = 1e-10;
const TAU: f64 = 0.05;
const RHO: f64 = 998.0;

#[test]
fn va6_a_sharp_front_in_plastic_pipe_shrinks_as_theory_says() {
    let g = STANDARD_GRAVITY.value();
    let c = RHO * g * ALPHA_D_OVER_E / 2.0;
    let kappa = A * c * J / (g * TAU);
    println!(
        "Theory: the front shrinks by exp(-{kappa:.5} x): half its height every {:.0} m",
        std::f64::consts::LN_2 / kappa
    );

    let wall = ViscoelasticWall::new(
        ALPHA_D_OVER_E,
        &[KelvinVoigt {
            compliance: J,
            retardation: Time::new(TAU),
        }],
    )
    .unwrap();
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(100.0),
    });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::new(50.0),
        closure: ValveClosure::Instantaneous,
        flow_coefficient: None,
    });
    net.add_pipe(r, v, pipe(L, D, A, 0.0).with_wall(wall))
        .unwrap();
    // Points 0, 100, ..., 600 m back from the valve (fraction from the
    // reservoir end).
    let distances = [0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0];
    let probes: Vec<Probe> = distances
        .iter()
        .map(|d| Probe::Pipe {
            pipe: net.pipe_id(0).unwrap(),
            fraction: (L - d) / L,
        })
        .collect();
    let q0 = Flow::new(V * std::f64::consts::PI * 0.25 * D * D);
    let run = net
        .simulate(
            &InitialCondition::PipeFlows(vec![q0]),
            Time::new(2.5),
            &probes,
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 4000,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let joukowsky = A * V / g;
    println!("  rigid pipe: every point would jump {joukowsky:.3} m (Joukowsky)");
    // Fit ln(jump) = ln(J0) - kappa x by least squares.
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for (k, &d) in distances.iter().enumerate() {
        let heads: Vec<f64> = run.trace.head(k).iter().map(|h| h.value()).collect();
        // The jump at the step the front arrives (the grid's wave speed is
        // within 1% of a, so look from just before d / a); behind the front
        // the creep brings part of the drop back, so later reads are higher.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
        let from = ((0.99 * d / A) / dt).floor() as usize;
        let at = (from..heads.len())
            .find(|&i| (heads[i] - heads[0]).abs() > 1e-9)
            .expect("the front arrives");
        let jump = heads[at] - heads[0];
        let expected = joukowsky * (-kappa * d).exp();
        println!(
            "  {d:>5.0} m from the valve: jump {jump:.3} m, theory {expected:.3} m ({:+.1}%)",
            (jump / expected - 1.0) * 100.0
        );
        let y = jump.ln();
        sx += d;
        sy += y;
        sxx += d * d;
        sxy += d * y;
    }
    #[allow(clippy::cast_precision_loss)] // seven points
    let n = distances.len() as f64;
    let measured = -(n * sxy - sx * sy) / (n * sxx - sx * sx);
    check_close("front damping rate kappa [1/m]", measured, kappa, 0.05);
    check(
        "the front has lost over half its height 200 m on",
        (-measured * 200.0_f64).exp() < 0.5,
    );
}
