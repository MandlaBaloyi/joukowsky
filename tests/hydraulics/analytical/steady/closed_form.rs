//! Problems simple enough to solve by hand.

use jk_hydraulics::{Network, Node, SteadyOptions, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY};
use jk_tests::{check_close, pipe};

use super::resistance;

/// Reservoir at 80 m feeds a consumer drawing 30 l/s through 1000 m of DN200.
/// The consumer's head is 80 - R Q^2.
#[test]
fn a_pipe_feeding_a_consumer() {
    let p = pipe(1000.0, 0.2, 1000.0, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let j = net.add_node(Node::Junction {
        demand: Flow::new(0.03),
    });
    net.add_pipe(r, j, p).unwrap();
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    println!("80 m reservoir -> 1000 m DN200 -> consumer drawing 30 l/s:");
    check_close("pipe flow [l/s]", s.pipe_flows[0].value() * 1e3, 30.0, 1e-9);
    check_close(
        "consumer head = 80 - R Q^2 [m]",
        s.node_heads[1].value(),
        80.0 - resistance(&p) * 0.03 * 0.03,
        1e-12,
    );
    println!("  solved in {} Newton rounds", s.iterations);
}

/// Two reservoirs, 80 m and 50 m, joined by one pipe: Q = sqrt(30 / R).
#[test]
fn two_reservoirs_and_one_pipe() {
    let p = pipe(2000.0, 0.25, 1000.0, 0.018);
    let mut net = Network::new(STANDARD_GRAVITY);
    let a = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let b = net.add_node(Node::Reservoir {
        head: Head::new(50.0),
    });
    net.add_pipe(a, b, p).unwrap();
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    println!("80 m and 50 m reservoirs joined by 2000 m of DN250:");
    check_close(
        "flow = sqrt(30 / R) [l/s]",
        s.pipe_flows[0].value() * 1e3,
        (30.0 / resistance(&p)).sqrt() * 1e3,
        1e-9,
    );
}

/// A valve discharging to air: the pipe loss and the valve loss share the
/// 80 m available, Q = sqrt(80 / (R + 1 / Kv^2)).
#[test]
fn a_valve_discharging_to_air() {
    let p = pipe(500.0, 0.15, 1000.0, 0.02);
    let kv = 0.004;
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Open,
        flow_coefficient: Some(kv),
    });
    net.add_pipe(r, v, p).unwrap();
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    println!("80 m reservoir -> 500 m DN150 -> valve (Kv = {kv}) to air:");
    let q = (80.0 / (resistance(&p) + 1.0 / (kv * kv))).sqrt();
    check_close("flow [l/s]", s.pipe_flows[0].value() * 1e3, q * 1e3, 1e-9);
    check_close(
        "head at the valve = Q^2 / Kv^2 [m]",
        s.node_heads[1].value(),
        q * q / (kv * kv),
        1e-9,
    );
}

/// Two different pipes in parallel carry 50 l/s between them. Both lose the
/// same head, so R1 Q1^2 = R2 Q2^2: the flow splits as Q1/Q2 = sqrt(R2/R1).
#[test]
fn parallel_pipes_split_the_flow_by_resistance() {
    let (p1, p2) = (
        pipe(800.0, 0.2, 1000.0, 0.02),
        pipe(1200.0, 0.15, 1000.0, 0.025),
    );
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let j = net.add_node(Node::Junction {
        demand: Flow::new(0.05),
    });
    net.add_pipe(r, j, p1).unwrap();
    net.add_pipe(r, j, p2).unwrap();
    let s = net.steady_state(&SteadyOptions::default()).unwrap();

    let (r1, r2) = (resistance(&p1), resistance(&p2));
    let share1 = r2.sqrt() / (r1.sqrt() + r2.sqrt());
    println!("50 l/s through two parallel pipes (DN200 800 m, DN150 1200 m):");
    check_close(
        "flow in pipe 1 [l/s]",
        s.pipe_flows[0].value() * 1e3,
        50.0 * share1,
        1e-9,
    );
    check_close(
        "flow in pipe 2 [l/s]",
        s.pipe_flows[1].value() * 1e3,
        50.0 * (1.0 - share1),
        1e-9,
    );
    check_close(
        "both lose the same head [m]",
        r1 * s.pipe_flows[0].value().powi(2),
        r2 * s.pipe_flows[1].value().powi(2),
        1e-9,
    );
    println!(
        "  residuals: energy {:.1e} m, continuity {:.1e} m^3/s",
        s.max_energy_residual.value(),
        s.max_continuity_residual.value()
    );
}
