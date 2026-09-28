//! Run-down with an exact answer.
//!
//! The pump faces a 90 m lift it cannot beat (shut-off head 80 m), so its
//! non-return valve stays shut and the flow is exactly zero. When the power
//! fails, only the shut-off torque slows it:
//!
//! ```text
//!   T = rho g (a0 alpha^2) Q_R / (2 eta_R w_R)
//!   I w_R d(alpha)/dt = -T      =>  d(alpha)/dt = -k alpha^2
//!   k = rho g a0 Q_R / (2 eta_R I w_R^2)
//!   alpha(t) = 1 / (1 + k (t - t_trip))
//! ```
//!
//! The simulated speed must follow that curve.

use jk_hydraulics::{GridOptions, InitialCondition, Probe, SteadyOptions};
use jk_kernel::{STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close};

use super::{station, station_pump};

#[test]
fn speed_follows_the_exact_run_down_curve() {
    let (net, pump_node, _) = station(100.0, Some(0.5));
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(10.0),
            &[Probe::Node(pump_node)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 2,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    let speeds = &run.pump_speeds[0].1;

    let pump = station_pump();
    let (a0, _, _) = pump.curve.coefficients();
    let w = pump.rated_omega();
    let k = 998.2 * STANDARD_GRAVITY.value() * a0 * pump.rated_flow.value()
        / (2.0 * pump.rated_efficiency * pump.inertia * w * w);

    // The run-down starts from the last sample still at full speed.
    #[allow(clippy::float_cmp)] // exactly 1.0 until the trip
    let start = speeds.iter().rposition(|&a| a == 1.0).unwrap();
    #[allow(clippy::cast_precision_loss)]
    let t_trip = start as f64 * dt;
    let mut worst = 0.0_f64;
    for (i, &alpha) in speeds.iter().enumerate().skip(start) {
        #[allow(clippy::cast_precision_loss)]
        let t = i as f64 * dt;
        worst = worst.max((alpha - 1.0 / (1.0 + k * (t - t_trip))).abs());
    }
    println!("Power fails at t = {t_trip:.2} s; k = {k:.4} /s; dt = {dt} s");
    println!(
        "  speed after 1 s: {:.4} (exact {:.4})",
        speeds[start + 100],
        1.0 / (1.0 + k)
    );
    println!("  speed after 9 s: {:.4}", speeds[speeds.len() - 1]);
    println!("  largest deviation from alpha = 1/(1 + k t): {worst:.1e}");
    check(
        "simulated run-down matches the exact curve (< 1e-5)",
        worst < 1e-5,
    );
    check_close(
        "flow stays exactly zero [l/s]",
        run.trace
            .flow(0)
            .iter()
            .map(|q| q.value().abs())
            .fold(0.0, f64::max),
        0.0,
        0.0,
    );
}
