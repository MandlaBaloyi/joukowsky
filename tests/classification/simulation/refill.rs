//! Refills: supply returning to a drained pipe after a cut, seen by a
//! logger at the pipe's closed far end (V-S6). The pipe: 500 m DN200,
//! f = 0.02, a = 1000 m/s, 30 m of supply, 50 m of water left in it.
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
//! the same pipe, which the MOC solver simulates - the test checks the two
//! speeds agree before using it. Its peak is the Joukowsky surge a v / g
//! PLUS line packing: the column was losing its 30 m of supply head to
//! friction along the way, and once stopped that head comes back on top.
//! The down-surges that follow fall below the vapour limit (-10 m) in the
//! simulation, which has no cavitation yet (S7) - not physical, so these
//! tests judge the first front only.

use jk_classification::{Context, EventClass};
use jk_hydraulics::refill::{Air, Refill, RefillRun, Vent, refill};
use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, Probe, SteadyOptions, ValveClosure,
};
use jk_kernel::{Head, Length, Pressure, STANDARD_GRAVITY, Time, WaveSpeed};
use jk_tests::{check, check_close, pipe};

use crate::verdict;

const L: f64 = 500.0;
const D: f64 = 0.2;
const F: f64 = 0.02;
const A: f64 = 1000.0;
const H: f64 = 30.0;
/// Truth recorded every 5 ms - finer than the logger's 7.8 ms.
const DT: f64 = 0.005;
/// Seconds the logger reads the drained pipe before the supply returns
/// (a real cut lasts hours; the supply watch needs a minute).
const EMPTY_FOR: f64 = 120.0;

fn refill_with(air: Air, duration: f64) -> RefillRun {
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

fn vented(mm: f64) -> Air {
    Air::Pocket {
        polytropic: 1.4,
        vent: Some(Vent {
            diameter: Length::new(mm / 1000.0),
            discharge: 0.6,
        }),
    }
}

/// The head at the far end after an instant stop of the pipe's free
/// discharge (reservoir 30 m -> pipe -> valve open to air, shut at t = 0),
/// and the pipe's steady velocity before it.
fn slam_at_the_end() -> (Vec<f64>, f64) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir { head: Head::new(H) });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Instantaneous,
        flow_coefficient: Some(1.0), // fully open: 1 cm of loss
    });
    net.add_pipe(r, v, pipe(L, D, A, F)).unwrap();
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
        .unwrap();
    let dt = run.trace.time_step().value();
    let head: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    // Steady velocity from the open valve: Q = Kv sqrt(H_valve).
    let area = std::f64::consts::PI * 0.25 * D * D;
    let velocity = head[0].sqrt() / area;
    // Resample on DT for joining onto the refill's record.
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

/// The logger's truth: `EMPTY_FOR` s of the drained pipe, then the air
/// head at the end while the water comes in, then (if it slammed) the
/// water hammer.
fn at_the_end(run: &RefillRun, after_slam: Option<&[f64]>) -> Vec<f64> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let lead = (EMPTY_FOR / DT) as usize;
    let mut truth = vec![0.0; lead];
    truth.extend(run.air_head.iter().map(|h| h.value()));
    if let Some(slam) = after_slam {
        truth.pop(); // the slam's own record is the wave's first sample
        truth.extend_from_slice(slam);
    }
    truth
}

#[test]
fn vs6_a_refill_through_a_large_air_valve_slams_and_is_called_a_refill() {
    println!("Supply returns to a drained 500 m pipe; a 100 mm air valve at the far end");
    println!("lets the air out, so the logger there reads ~0 until the water slams in:");
    let run = refill_with(vented(100.0), 300.0);
    let slam = run.slam.expect("the column reaches the end");
    let (hammer, open_velocity) = slam_at_the_end();
    println!(
        "  the column arrives after {:.1} s at {:.2} m/s",
        slam.at.value(),
        slam.velocity.value()
    );
    check_close(
        "  it arrives at the speed the open pipe would carry [m/s]",
        slam.velocity.value(),
        open_velocity,
        0.02,
    );
    let peak = hammer.iter().copied().fold(f64::MIN, f64::max);
    check_close(
        "  the slam the logger sees: Joukowsky a v / g + the supply head regained (line packing) [m]",
        peak,
        slam.surge.value() + H,
        0.01,
    );
    let c = verdict(
        &at_the_end(&run, Some(&hammer)),
        DT,
        &Context::unknown(),
        11,
    );
    check("refill", c.class == EventClass::Refill);
    check("at least 99% confident", c.confidence >= 0.99);
}

#[test]
fn vs6_a_refill_against_trapped_air_is_called_a_refill() {
    for mm in [5.0, 20.0] {
        println!(
            "Supply returns to the drained pipe; a {mm} mm air valve keeps most of the air \
             in, so the logger at the end sees it squeezed over a minute - too slow for a \
             front:"
        );
        let run = refill_with(vented(mm), 300.0);
        let after = run.slam.map(|_| slam_at_the_end().0);
        let truth = at_the_end(&run, after.as_deref());
        let c = verdict(&truth, DT, &Context::unknown(), 12);
        check(
            "  the first front comes once the pipe is already under pressure",
            c.features.level_before.value() > 2.0,
        );
        check(
            "  yet with the supply watch it is called a refill",
            c.class == EventClass::Refill,
        );
    }
}

#[test]
fn vs6_the_same_slam_on_a_pipe_under_pressure_is_not_a_refill() {
    println!("Control: the same valve slamming shut, but the pipe was full and under");
    println!("pressure before (sensor halfway along it, ~15 m) - not a refill:");
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir { head: Head::new(H) });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Instantaneous,
        flow_coefficient: Some(1.0),
    });
    let p = net.add_pipe(r, v, pipe(L, D, A, F)).unwrap();
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(6.0),
            &[Probe::Pipe {
                pipe: p,
                fraction: 0.5,
            }],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let h: Vec<f64> = run.trace.head(0).iter().map(|h| h.value()).collect();
    let c = verdict(
        &crate::with_quiet_lead(&h, dt, 3.0),
        dt,
        &Context::unknown(),
        13,
    );
    check("valve closure", c.class == EventClass::ValveClosure);
}
