//! A pump trip on a rising main - load-shedding, as the engine will see it.
//!
//! The pump delivers into a 2000 m rising main against a 50 m lift. At
//! t = 1 s the power fails. What must happen:
//!
//!   - before the trip nothing moves (the steady state is exact);
//!   - the pump slows; the flow falls; the non-return valve never lets it
//!     reverse, and shuts once the pump can no longer push;
//!   - the falling flow sends a DOWN-SURGE up the main. In a frictionless
//!     main, until the wave has been to the top and back (2L/a = 4 s), the
//!     discharge head must follow the Joukowsky characteristic exactly:
//!     `H_d` - `H_d0` = B (Q - Q0), B = a / (g A).
//!
//! The depth of that down-surge is what breaks pipes and draws in dirty
//! water; the docs call for it to be pre-computed for every pump
//! (`outages_scale_and_errors.txt` 1a).

use jk_hydraulics::{GridOptions, InitialCondition, NetworkRun, Probe, SteadyOptions};
use jk_kernel::{STANDARD_GRAVITY, Time};
use jk_tests::check;

use super::{station_pump, station_with_friction};

const T_TRIP: f64 = 1.0;

/// Runs the station with a trip at `T_TRIP`; probes: pump, mid-main.
fn run_trip(friction: f64) -> (NetworkRun, f64) {
    let (net, pump, main) = station_with_friction(60.0, Some(T_TRIP), friction);
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(14.0),
            &[
                Probe::Node(pump),
                Probe::Pipe {
                    pipe: main,
                    fraction: 0.5,
                },
            ],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 2,
            },
        )
        .unwrap();
    let p = net.pipes()[main.index()].pipe;
    let b = p.wave_speed().value() / (STANDARD_GRAVITY.value() * p.area().value());
    (run, b)
}

#[test]
fn pump_trip_on_a_rising_main() {
    let (run, b) = run_trip(0.018);
    let t = &run.trace;
    let heads: Vec<f64> = t.head(0).iter().map(|h| h.value()).collect();
    let flows: Vec<f64> = t.flow(0).iter().map(|q| q.value()).collect();
    let speeds = &run.pump_speeds[0].1;
    let (h0, q0) = (heads[0], flows[0]);
    let k_trip = t.sample_at(Time::new(T_TRIP));

    println!(
        "Before the trip: {:.1} l/s at a discharge head of {h0:.2} m",
        q0 * 1e3
    );
    let still = heads[..k_trip]
        .iter()
        .map(|h| (h - h0).abs())
        .fold(0.0, f64::max);
    check("nothing moves before the trip (< 1e-9 m)", still < 1e-9);

    println!("After the power fails at t = {T_TRIP} s:");
    let at = |s: f64| t.sample_at(Time::new(T_TRIP + s));
    for s in [0.5, 1.0, 2.0] {
        println!(
            "  +{s} s: speed {:.0}%, flow {:.1} l/s",
            speeds[at(s)] * 100.0,
            flows[at(s)] * 1e3
        );
    }
    #[allow(clippy::float_cmp)] // the non-return valve sets exactly 0.0
    let shut = flows
        .iter()
        .skip(k_trip)
        .position(|&q| q == 0.0)
        .map(|i| i + k_trip);
    if let Some(k) = shut {
        println!("  non-return valve shuts at t = {:.2} s", t.time(k).value());
    }
    check("the non-return valve shuts", shut.is_some());
    check("flow never reverses", flows.iter().all(|&q| q >= 0.0));

    // The model's valid zone: normal pumping, where the pump ADDS head
    // (H(Q, alpha) >= 0). Deep in the down-surge the discharge head can fall
    // below the suction head; water is then dragged through the pump and
    // drives it forward like a windmill - a different zone that needs
    // four-quadrant curves (version 2). Speed must fall while in the zone.
    let curve = station_pump().curve;
    let in_zone: Vec<bool> = flows
        .iter()
        .zip(speeds)
        .map(|(&q, &a)| curve.head(q, a) >= 0.0)
        .collect();
    let falls_in_zone = speeds
        .windows(2)
        .zip(in_zone.windows(2))
        .all(|(s, z)| !(z[0] && z[1]) || s[1] <= s[0]);
    check("speed never rises while the pump is pumping", falls_in_zone);
    match in_zone.iter().position(|&z| !z) {
        Some(k) => println!(
            "  pump LEAVES its normal zone at t = {:.2} s (discharge below suction):\n  \
             beyond this point its speed needs four-quadrant curves (version 2)",
            t.time(k).value()
        ),
        None => println!("  pump stays in its normal zone throughout"),
    }

    let (min_pump, min_mid) = (t.min_head(0).value(), t.min_head(1).value());
    println!("Down-surge:");
    println!("  lowest head at the pump      {min_pump:.1} m (steady {h0:.1} m)");
    println!("  lowest head mid-main         {min_mid:.1} m");
    println!(
        "  Joukowsky bound for stopping {:.1} l/s: B Q0 = {:.1} m",
        q0 * 1e3,
        b * q0
    );
    if min_pump < -8.0 || min_mid < -8.0 {
        println!("  WARNING: the head falls below the vapour floor (~ -8 to -10 m gauge):");
        println!("  in reality the water column would SEPARATE and its collapse can exceed");
        println!("  Joukowsky. This main needs surge protection. Cavitation is not modelled");
        println!("  yet (milestone S7), so heads below the floor are not physical.");
    }
}

#[test]
fn frictionless_down_surge_follows_the_joukowsky_characteristic() {
    let (run, b) = run_trip(0.0);
    let t = &run.trace;
    let heads: Vec<f64> = t.head(0).iter().map(|h| h.value()).collect();
    let flows: Vec<f64> = t.flow(0).iter().map(|q| q.value()).collect();
    let (h0, q0) = (heads[0], flows[0]);
    let dt = t.time_step().value();
    let k_trip = t.sample_at(Time::new(T_TRIP));
    let k_end = t.sample_at(Time::new(T_TRIP + 4.0 - 2.0 * dt)); // before 2L/a
    let worst = (k_trip..k_end)
        .map(|k| ((heads[k] - h0) - b * (flows[k] - q0)).abs())
        .fold(0.0, f64::max);
    println!("Frictionless main, B = {b:.1} s/m^2, first 2L/a = 4 s after the trip:");
    println!("  largest gap between H_d - H_d0 and B (Q - Q0): {worst:.1e} m");
    check(
        "discharge head follows the Joukowsky characteristic (< 1e-9 m)",
        worst < 1e-9,
    );
}
