//! The drained main: supply returning to an emptied pipe after a cut, as
//! a logger at its closed far end sees it (V-S6, shared by the
//! classification and pipeline tests). The pipe: 500 m DN200, f = 0.02,
//! a = 1000 m/s, 30 m of supply, 50 m of water left in it.
//!
//! Before the supply returns the logger reads about 0 (air at atmospheric
//! pressure). Then, depending on the air valve at the end (FN-032):
//! - a small valve keeps the air in: the logger sees the air being squeezed
//!   - a gradual rise from 0 (rigid-column model, hydraulics `refill.rs`);
//! - a large valve lets the air out: the logger reads 0 until the water
//!   arrives and stops dead - the slam.
//!
//! What the logger sees AFTER the slam is a water-hammer wave: the column,
//! by then flowing at nearly the speed the open pipe would carry
//! (friction balancing the supply), is stopped at the closed end. That is
//! exactly an instant closure of a valve discharging to air at the end of
//! the same pipe, which the MOC solver simulates - the tests check the two
//! speeds agree. Its peak is the Joukowsky surge a v / g PLUS line
//! packing: the column was losing its 30 m of supply head to friction
//! along the way, and once stopped that head comes back on top. The
//! down-surges that follow fall below the vapour limit (-10 m) in the
//! simulation, which has no cavitation yet (S7) - not physical, so tests
//! judge the first front only.

use jk_hydraulics::refill::{Air, Refill, RefillRun, Vent, refill};
use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, Probe, SteadyOptions, ValveClosure,
};
use jk_kernel::{Head, Length, Pressure, STANDARD_GRAVITY, Time, WaveSpeed};

use crate::pipe;

/// Length [m].
pub const L: f64 = 500.0;
/// Diameter [m].
pub const D: f64 = 0.2;
/// Darcy friction factor.
pub const F: f64 = 0.02;
/// Wave speed [m/s].
pub const A: f64 = 1000.0;
/// Supply head [m].
pub const H: f64 = 30.0;
/// Truth recorded every 5 ms - finer than a 128 S/s logger's 7.8 ms.
pub const DT: f64 = 0.005;
/// Seconds the logger reads the drained pipe before the supply returns
/// (a real cut lasts hours; the supply watch needs a minute).
pub const EMPTY_FOR: f64 = 120.0;

/// The refill with `air` ahead of the column, for up to `duration` s.
///
/// # Panics
/// Never for the fixed pipe (a test bug otherwise).
#[must_use]
pub fn refill_with(air: Air, duration: f64) -> RefillRun {
    refill(
        &Refill {
            length: Length::new(L),
            diameter: Length::new(D),
            friction: F,
            wave_speed: WaveSpeed::new(A),
            rise: Length::ZERO,
            entrance_loss: 0.0,
            supply: Head::new(H),
            start: Length::new(50.0),
            atmosphere: Pressure::new(101_325.0),
            air,
            gravity: STANDARD_GRAVITY,
        },
        Time::new(duration),
        Time::new(DT),
    )
    .expect("a physical refill")
}

/// Adiabatic air behind an air valve of `mm` millimetres (Cd 0.6).
#[must_use]
pub fn vented(mm: f64) -> Air {
    Air::Pocket {
        polytropic: 1.4,
        vent: Some(Vent {
            diameter: Length::new(mm / 1000.0),
            discharge: 0.6,
        }),
    }
}

/// The free-discharging pipe: reservoir 30 m -> pipe -> valve open to air
/// (fully open: 1 cm of loss), shut instantly at t = 0.
///
/// # Panics
/// Never for the fixed pipe (a test bug otherwise).
#[must_use]
pub fn free_discharge() -> (Network, jk_hydraulics::NodeId, jk_hydraulics::PipeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir { head: Head::new(H) });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Instantaneous,
        flow_coefficient: Some(1.0),
    });
    let p = net.add_pipe(r, v, pipe(L, D, A, F)).expect("valid pipe");
    (net, v, p)
}

/// The head at the far end for 6 s after an instant stop of the pipe's
/// free discharge, every [`DT`], and the pipe's steady velocity before it.
///
/// # Panics
/// Never for the fixed pipe (a test bug otherwise).
#[must_use]
pub fn slam_at_the_end() -> (Vec<f64>, f64) {
    let (net, v, _) = free_discharge();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(6.0),
            &[Probe::Node(v)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .expect("simulation runs");
    let dt = run.trace.time_step().value();
    let head: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    // Steady velocity from the open valve: Q = Kv sqrt(H_valve).
    let area = std::f64::consts::PI * 0.25 * D * D;
    let velocity = head[0].sqrt() / area;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (6.0 / DT) as usize;
    let at = |t: f64| {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let i = ((t / dt) as usize).min(head.len() - 2);
        #[allow(clippy::cast_precision_loss)]
        let w = t / dt - i as f64;
        head[i] + w * (head[i + 1] - head[i])
    };
    #[allow(clippy::cast_precision_loss)]
    let resampled = (0..n).map(|k| at(k as f64 * DT)).collect();
    (resampled, velocity)
}

/// The logger's truth for `run`: [`EMPTY_FOR`] s of the drained pipe, then
/// the air head at the end while the water comes in, then (if it slammed)
/// the water hammer - every [`DT`].
#[must_use]
pub fn at_the_end(run: &RefillRun) -> Vec<f64> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let lead = (EMPTY_FOR / DT) as usize;
    let mut truth = vec![0.0; lead];
    truth.extend(run.air_head.iter().map(|h| h.value()));
    if run.slam.is_some() {
        truth.pop(); // the slam's own record is the wave's first sample
        truth.extend(slam_at_the_end().0);
    }
    truth
}
