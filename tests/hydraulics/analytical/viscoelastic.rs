//! V-A5: a plastic (viscoelastic) pipe wall slows and damps the pipe's ring
//! exactly as theory says (milestone S6, `pipe/viscoelastic.rs`).
//!
//! Reservoir -> 400 m of 100 mm plastic pipe -> valve, no friction, 0.2 m/s;
//! the valve slams shut. A rigid pipe would ring for ever at the quarter-
//! wave period 4L/a. A creeping wall does two things, both exact for small
//! waves: with each Kelvin-Voigt element (compliance J, retardation tau),
//! the Laplace-transformed equations give, for the ring's complex frequency
//! s = -delta + i omega,
//!
//! ```text
//!   s^2 (1 + phi(s)) + omega0^2 = 0,   phi(s) = (2 a^2 c / g) J / (1 + s tau),
//!   omega0 = pi a / (2 L),   c = rho g alpha D / (2 e)
//! ```
//!
//! - the ring is SLOWER: omega < omega0 (the wall's delayed give makes the
//!   pipe effectively softer);
//! - it DIES AWAY at the rate delta.
//!
//! The test solves that equation (Newton's method on complex numbers) and
//! measures both in the simulation. Wall figures are illustrative for PE
//! [VERIFY against Covas et al. 2005 and field data]: a = 400 m/s,
//! alpha D / e = 16.7, J = 1e-10 1/Pa, tau = 0.05 s.

use jk_hydraulics::pipe::viscoelastic::{KelvinVoigt, ViscoelasticWall};
use jk_hydraulics::{GridOptions, InitialCondition, Network, Node, Probe, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const L: f64 = 400.0;
const A: f64 = 400.0;
const D: f64 = 0.1;
const V: f64 = 0.2;
const ALPHA_D_OVER_E: f64 = 16.7;
const J: f64 = 1e-10;
const TAU: f64 = 0.05;
const G: f64 = 9.806_65;
const RHO: f64 = 998.0;

/// Complex numbers, just enough for Newton's method.
#[derive(Clone, Copy, Debug)]
struct C(f64, f64);

impl C {
    fn add(self, o: C) -> C {
        C(self.0 + o.0, self.1 + o.1)
    }
    fn mul(self, o: C) -> C {
        C(self.0 * o.0 - self.1 * o.1, self.0 * o.1 + self.1 * o.0)
    }
    fn div(self, o: C) -> C {
        let d = o.0 * o.0 + o.1 * o.1;
        C(
            (self.0 * o.0 + self.1 * o.1) / d,
            (self.1 * o.0 - self.0 * o.1) / d,
        )
    }
    fn scale(self, k: f64) -> C {
        C(self.0 * k, self.1 * k)
    }
}

/// The ring's complex frequency s = -delta + i omega, from theory.
fn theory() -> (f64, f64) {
    let omega0 = std::f64::consts::PI * A / (2.0 * L);
    let c = RHO * G * ALPHA_D_OVER_E / 2.0;
    let k = 2.0 * A * A * c / G * J;
    let one = C(1.0, 0.0);
    let mut s = C(0.0, omega0);
    for _ in 0..50 {
        let den = one.add(s.scale(TAU));
        let phi = C(k, 0.0).div(den);
        let dphi = C(-k * TAU, 0.0).div(den.mul(den));
        let f = s.mul(s).mul(one.add(phi)).add(C(omega0 * omega0, 0.0));
        let df = s.scale(2.0).mul(one.add(phi)).add(s.mul(s).mul(dphi));
        s = s.add(f.div(df).scale(-1.0));
    }
    (-s.0, s.1)
}

/// Head at the valve after it slams shut [m], every time step.
fn simulate(plastic: bool, seconds: f64, reaches: u32) -> (Vec<f64>, f64) {
    let mut p = pipe(L, D, A, 0.0);
    if plastic {
        let wall = ViscoelasticWall::new(
            ALPHA_D_OVER_E,
            &[KelvinVoigt {
                compliance: J,
                retardation: Time::new(TAU),
            }],
        )
        .unwrap();
        p = p.with_wall(wall);
    }
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(100.0),
    });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::new(50.0),
        closure: ValveClosure::Instantaneous,
        flow_coefficient: None,
    });
    net.add_pipe(r, v, p).unwrap();
    let q0 = Flow::new(V * std::f64::consts::PI * 0.25 * D * D);
    let run = net
        .simulate(
            &InitialCondition::PipeFlows(vec![q0]),
            Time::new(seconds),
            &[Probe::Node(v)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: reaches,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    (run.trace.head(0).iter().map(|h| h.value()).collect(), dt)
}

/// Amplitude and phase of the component at frequency `omega` over one
/// period starting at time `t0`, of the ring around 100 m.
fn component(heads: &[f64], dt: f64, omega: f64, t0: f64) -> (f64, f64) {
    let period = std::f64::consts::TAU / omega;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive
    let (first, n) = ((t0 / dt).round() as usize, (period / dt).round() as usize);
    let (mut re, mut im) = (0.0, 0.0);
    for (k, h) in heads.iter().enumerate().skip(first).take(n) {
        #[allow(clippy::cast_precision_loss)] // step counts are small
        let t = k as f64 * dt;
        let x = h - 100.0;
        re += x * (omega * t).cos();
        im -= x * (omega * t).sin();
    }
    #[allow(clippy::cast_precision_loss)]
    let scale = 2.0 / n as f64;
    ((re * scale).hypot(im * scale), im.atan2(re))
}

#[test]
fn va5_a_creeping_wall_slows_and_damps_the_ring_as_theory_says() {
    let (delta, omega) = theory();
    let omega0 = std::f64::consts::PI * A / (2.0 * L);
    let period = std::f64::consts::TAU / omega;
    println!(
        "Theory: rigid ring {:.3} s period; with the creeping wall {period:.3} s, dying away at \
         {delta:.4} per second (amplitude x{:.2} per period)",
        std::f64::consts::TAU / omega0,
        (-delta * period).exp()
    );

    // The creep is integrated along each characteristic (trapezoid rule),
    // so the decay rate must be close to theory even on a coarse grid and
    // closer on finer ones. (The first version took the creep at the grid
    // point over each step: +12.7% at 100 reaches, first order - FN-021.)
    let seconds = 12.0 * period;
    let measure = |reaches: u32| {
        let (heads, dt) = simulate(true, seconds, reaches);
        // The fundamental over periods 2-3 and 9-10: the faster, more
        // strongly damped overtones have faded by then.
        let (early, phase_early) = component(&heads, dt, omega, 2.0 * period);
        let (late, phase_late) = component(&heads, dt, omega, 9.0 * period);
        (
            (early / late).ln() / (7.0 * period),
            (phase_late - phase_early).abs(),
            dt,
        )
    };
    let (d100, _, _) = measure(100);
    let (d200, _, _) = measure(200);
    let (d400, drift, dt) = measure(400);
    for (n, d) in [(100, d100), (200, d200), (400, d400)] {
        println!(
            "  {n} reaches: decay {d:.6} per second, {:+.1}% from theory",
            (d / delta - 1.0) * 100.0
        );
    }
    check_close("decay rate, 100 reaches [1/s]", d100, delta, 0.01);
    check_close("decay rate, 400 reaches [1/s]", d400, delta, 0.003);
    check(
        "finer grids come closer",
        (d400 - delta).abs() <= (d100 - delta).abs(),
    );
    println!(
        "  phase of the ring at theory's frequency moves {drift:.3} rad over 7 periods \
         (0 if the frequency is right)"
    );
    check(
        "the ring keeps theory's frequency (drift < 0.1 rad)",
        drift < 0.1,
    );

    let (rigid, _) = simulate(false, seconds, 400);
    let (r_early, _) = component(&rigid, dt, omega0, 2.0 * std::f64::consts::TAU / omega0);
    let (r_late, _) = component(&rigid, dt, omega0, 9.0 * std::f64::consts::TAU / omega0);
    println!(
        "  rigid pipe, same run: fundamental {r_early:.3} m then {r_late:.3} m (no creep, no friction: no decay)"
    );
    check_close("rigid pipe does not decay", r_late, r_early, 0.01);
}
