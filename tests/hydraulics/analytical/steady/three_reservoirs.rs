//! The classic three-reservoir problem (every hydraulics textbook has it).
//!
//! Reservoirs at 100 m, 80 m and 40 m each connect to one junction. Which
//! way does water flow in the middle pipe, and how much? The junction head
//! H must make the flows balance:
//!
//! ```text
//!   sum_i  sign(H_i - H) sqrt(|H_i - H| / R_i)  =  0
//! ```
//!
//! The left side falls steadily as H rises, so a plain bisection search
//! finds H to machine precision - an answer found completely independently
//! of the engine's Newton method.

use jk_hydraulics::{Network, Node, SteadyOptions};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY};
use jk_tests::{check, check_close, pipe};

use super::resistance;

#[test]
fn three_reservoirs_meeting_at_one_junction() {
    let levels = [100.0, 80.0, 40.0];
    let pipes = [
        pipe(1500.0, 0.3, 1000.0, 0.02),
        pipe(900.0, 0.2, 1000.0, 0.02),
        pipe(1200.0, 0.25, 1000.0, 0.02),
    ];
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = net.add_node(Node::Junction { demand: Flow::ZERO });
    for (level, p) in levels.iter().zip(&pipes) {
        let r = net.add_node(Node::Reservoir {
            head: Head::new(*level),
        });
        net.add_pipe(r, j, *p).unwrap();
    }
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    // Independent answer: bisection on the junction head.
    let r: Vec<f64> = pipes.iter().map(resistance).collect();
    let inflow = |h: f64| -> f64 {
        levels
            .iter()
            .zip(&r)
            .map(|(hi, ri)| (hi - h).signum() * ((hi - h).abs() / ri).sqrt())
            .sum()
    };
    let (mut lo, mut hi) = (40.0, 100.0);
    for _ in 0..200 {
        let mid = f64::midpoint(lo, hi);
        if inflow(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let h_ref = f64::midpoint(lo, hi);

    println!("Reservoirs at 100, 80 and 40 m meeting at one junction:");
    check_close(
        "junction head: engine vs bisection [m]",
        s.node_heads[j.index()].value(),
        h_ref,
        1e-12,
    );
    for (i, level) in levels.iter().enumerate() {
        let q_ref = (level - h_ref).signum() * ((level - h_ref).abs() / r[i]).sqrt();
        check_close(
            &format!("flow from the {level} m reservoir [l/s]"),
            s.pipe_flows[i].value() * 1e3,
            q_ref * 1e3,
            1e-9,
        );
    }
    let middle = s.pipe_flows[1].value();
    println!(
        "  the 80 m reservoir is {} ({:.2} l/s)",
        if middle > 0.0 {
            "SUPPLYING the junction"
        } else {
            "being FILLED"
        },
        middle.abs() * 1e3
    );
    check(
        "water balances at the junction",
        s.max_continuity_residual.value() < 1e-12,
    );
    println!("  solved in {} Newton rounds", s.iterations);
}
