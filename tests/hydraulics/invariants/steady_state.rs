//! If nothing moves, nothing may change.
//!
//! A solver that drifts away from a steady state on its own would invent
//! transients that are not there - false alarms built into the physics.
//! With the valve left open, 20 s of simulation must reproduce the steady
//! state (including the friction head gradient) to rounding error.

use jk_hydraulics::ValveClosure;
use jk_kernel::Time;
use jk_tests::{check, check_close, standard_system};

#[test]
fn an_undisturbed_pipe_stays_steady() {
    let s = standard_system(0.02, ValveClosure::Open, 25);
    let probes = [0, 12, s.valve_node()];
    let trace = s.simulate(Time::new(20.0), &probes).unwrap();

    println!("Steady state with friction f = 0.02:");
    check_close(
        "valve head: solver vs hand formula [m]",
        trace.head(2)[0].value(),
        s.steady_valve_head().value(),
        1e-12,
    );

    for (j, name) in ["reservoir end", "mid pipe", "valve"].iter().enumerate() {
        let (h0, q0) = (trace.head(j)[0].value(), trace.flow(j)[0].value());
        let head_drift = trace
            .head(j)
            .iter()
            .map(|h| (h.value() - h0).abs())
            .fold(0.0, f64::max);
        let flow_drift = trace
            .flow(j)
            .iter()
            .map(|q| (q.value() - q0).abs())
            .fold(0.0, f64::max);
        println!(
            "  {name}: largest head drift {head_drift:.1e} m, flow drift {flow_drift:.1e} m^3/s"
        );
        check(
            &format!("{name} holds steady"),
            head_drift < 1e-9 && flow_drift < 1e-12,
        );
    }
}
