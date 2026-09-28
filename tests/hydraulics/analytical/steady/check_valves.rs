//! Check valves in the steady state: open when the flow runs their way,
//! shut when it would run back - found by the solver, not told to it.
//!
//! ```text
//!   left reservoir --500 m--> [check valve] --500 m--> right reservoir
//! ```

use jk_hydraulics::{Network, Node, SteadyOptions};
use jk_kernel::{Head, STANDARD_GRAVITY};
use jk_tests::{check, check_close, pipe};

use super::resistance;

fn solve(left: f64, right: f64) -> (jk_hydraulics::SteadySolution, jk_hydraulics::NodeId, f64) {
    let p = pipe(500.0, 0.2, 1000.0, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let a = net.add_node(Node::Reservoir {
        head: Head::new(left),
    });
    let cv = net.add_node(Node::CheckValve);
    let b = net.add_node(Node::Reservoir {
        head: Head::new(right),
    });
    net.add_pipe(a, cv, p).unwrap();
    net.add_pipe(cv, b, p).unwrap();
    (
        net.steady_state(&SteadyOptions::default()).unwrap(),
        cv,
        resistance(&p),
    )
}

#[test]
fn forward_head_opens_the_valve() {
    let (s, _, r) = solve(80.0, 50.0);
    println!("80 m on the valve's upstream side, 50 m downstream:");
    check("valve is open", s.closed_check_valves.is_empty());
    check_close(
        "flow = sqrt(30 / 2R) [l/s]",
        s.pipe_flows[0].value() * 1e3,
        (30.0 / (2.0 * r)).sqrt() * 1e3,
        1e-9,
    );
}

#[test]
fn reverse_head_shuts_the_valve() {
    let (s, cv, _) = solve(50.0, 80.0);
    println!("50 m upstream, 80 m downstream - water would run backwards:");
    check("valve is shut", s.closed_check_valves == vec![cv]);
    check_close("flow [l/s]", s.pipe_flows[0].value() * 1e3, 0.0, 0.0);
    check_close(
        "upstream side keeps the 50 m head [m]",
        s.node_heads[cv.index()].value(),
        50.0,
        1e-9,
    );
}
