//! Refills: supply returning to a drained pipe after a cut, seen by a
//! logger at the pipe's closed far end (V-S6). The drained main and what
//! the logger sees are described in `tests/common/refill.rs`.

use jk_classification::{Context, EventClass};
use jk_hydraulics::{GridOptions, InitialCondition, Probe, SteadyOptions};
use jk_kernel::Time;
use jk_tests::refill::{DT, H, at_the_end, free_discharge, refill_with, slam_at_the_end, vented};
use jk_tests::{check, check_close};

use crate::verdict;

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
    let c = verdict(&at_the_end(&run), DT, &Context::unknown(), 11);
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
        let c = verdict(&at_the_end(&run), DT, &Context::unknown(), 12);
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
    let (net, _, p) = free_discharge();
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
