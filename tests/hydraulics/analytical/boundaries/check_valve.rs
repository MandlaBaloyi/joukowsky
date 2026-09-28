//! Check valve: a one-way door between two pipes. The story of a pump trip
//! from the valve's point of view, one time step at a time.

use jk_hydraulics::boundary::check_valve::{CheckValveState, step};
use jk_tests::{check, check_close};

const B: f64 = 5000.0;

#[test]
fn the_door_opens_one_way_only() {
    let (h0, q0) = (60.0, 0.03);
    println!("1. Normal operation: forward flow {q0} m^3/s at {h0} m on both sides.");
    let s = step(CheckValveState::Open, h0 + B * q0, B, h0 - B * q0, B);
    check("valve open", s.state == CheckValveState::Open);
    check_close("flow passes unchanged [m^3/s]", s.flow, q0, 1e-15);

    println!("2. Pump trips: upstream pressure collapses, flow tries to run back.");
    let s = step(CheckValveState::Open, 30.0, B, 65.0, B);
    check("valve shuts", s.state == CheckValveState::Closed);
    check_close("no flow through", s.flow, 0.0, 0.0);
    println!(
        "  upstream side {:.1} m, downstream side {:.1} m: each is now a dead end",
        s.head_upstream, s.head_downstream
    );

    println!("3. Downstream still higher: the valve holds.");
    let s = step(CheckValveState::Closed, 50.0, B, 65.0, B);
    check("valve stays shut", s.state == CheckValveState::Closed);

    println!("4. Pump restarts and upstream pushes harder: the valve reopens.");
    let s = step(CheckValveState::Closed, 90.0, B, 65.0, B);
    check("valve reopens", s.state == CheckValveState::Open);
    check("flow is forward", s.flow > 0.0);
}
