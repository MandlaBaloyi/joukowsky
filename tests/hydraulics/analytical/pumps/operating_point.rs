//! Where the pump runs: the pump curve meets the system curve.
//!
//! The pump adds H(Q) = a0 + a1 Q + a2 Q^2; the system needs the static lift
//! plus friction, (top - 10) + R Q^2. Equal at the operating point:
//!
//! ```text
//!   (R - a2) Q^2 - a1 Q - (a0 - lift) = 0
//! ```
//!
//! a quadratic with one positive root - solved here by hand.

use jk_hydraulics::SteadyOptions;
use jk_tests::{check, check_close};

use super::{station, station_pump};
use crate::analytical::steady::resistance;

#[test]
fn pump_curve_meets_system_curve() {
    let (net, pump, main) = station(60.0, None);
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    let (a0, a1, a2) = station_pump().curve.coefficients();
    let r: f64 = net.pipes().iter().map(|p| resistance(&p.pipe)).sum();
    let lift = 60.0 - 10.0;
    let a = r - a2;
    let q = (a1 + (a1 * a1 + 4.0 * a * (a0 - lift)).sqrt()) / (2.0 * a);

    println!("Sump 10 m -> pump -> 2000 m rising main -> reservoir 60 m:");
    check_close(
        "flow: engine vs hand [l/s]",
        s.pipe_flows[main.index()].value() * 1e3,
        q * 1e3,
        1e-9,
    );
    let pump_lift = s
        .pump_lifts
        .iter()
        .find(|(id, _)| *id == pump)
        .unwrap()
        .1
        .value();
    check_close(
        "pump head = curve at that flow [m]",
        pump_lift,
        a0 + a1 * q + a2 * q * q,
        1e-9,
    );
    println!("  static lift {lift} m + friction {:.2} m", r * q * q);
    check("the pump is delivering", s.idle_pumps.is_empty());
}

#[test]
fn a_pump_cannot_beat_a_lift_above_its_shut_off_head() {
    let (net, pump, main) = station(100.0, None);
    let s = net.steady_state(&SteadyOptions::default()).unwrap();
    println!("Lift 90 m against a pump whose shut-off head is 80 m:");
    check(
        "the non-return valve holds it idle",
        s.idle_pumps == vec![pump],
    );
    check_close(
        "no flow [l/s]",
        s.pipe_flows[main.index()].value() * 1e3,
        0.0,
        0.0,
    );
}
