//! The steady solver and the transient solver must agree about "nothing
//! happening".
//!
//! The steady solver (S4) finds the flows and heads of a looped network
//! with consumers, a check valve and an outlet valve. The transient solver
//! (S3) then starts from that state with nothing disturbing it. If the
//! steady answer were even slightly off, the transient solver would release
//! the error as a fake pressure wave - a false alarm built into the physics.
//!
//! ```text
//!   R(90 m) --600 m-- A --400 m-- B (10 l/s)
//!                     |           |
//!                   500 m       300 m          (a loop: A-B-C)
//!                     |           |
//!                     C (15 l/s) -+
//!                     |
//!                   200 m -- check valve -- 200 m -- outlet valve (Kv 0.004)
//! ```

use jk_hydraulics::{
    GridOptions, InitialCondition, Network, Node, Probe, SteadyOptions, ValveClosure,
};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, pipe};

#[test]
fn a_solved_steady_state_stays_steady_in_the_transient_solver() {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(90.0),
    });
    let a = net.add_node(Node::Junction { demand: Flow::ZERO });
    let b = net.add_node(Node::Junction {
        demand: Flow::new(0.010),
    });
    let c = net.add_node(Node::Junction {
        demand: Flow::new(0.015),
    });
    let cv = net.add_node(Node::CheckValve);
    let v = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Open,
        flow_coefficient: Some(0.004),
    });
    net.add_pipe(r, a, pipe(600.0, 0.3, 1100.0, 0.02)).unwrap();
    net.add_pipe(a, b, pipe(400.0, 0.2, 1100.0, 0.02)).unwrap();
    net.add_pipe(a, c, pipe(500.0, 0.2, 1100.0, 0.02)).unwrap();
    let loop_pipe = net.add_pipe(b, c, pipe(300.0, 0.15, 1100.0, 0.02)).unwrap();
    net.add_pipe(c, cv, pipe(200.0, 0.15, 1100.0, 0.02))
        .unwrap();
    net.add_pipe(cv, v, pipe(200.0, 0.15, 1100.0, 0.02))
        .unwrap();

    let steady = net.steady_state(&SteadyOptions::default()).unwrap();
    println!("Steady solve: {} Newton rounds", steady.iterations);
    println!(
        "  residuals: energy {:.1e} m, continuity {:.1e} m^3/s",
        steady.max_energy_residual.value(),
        steady.max_continuity_residual.value()
    );
    for (name, node) in [("A", a), ("B", b), ("C", c), ("outlet valve", v)] {
        println!(
            "  head at {name:<13} {:.3} m",
            steady.node_heads[node.index()].value()
        );
    }
    println!(
        "  loop pipe B->C carries {:.3} l/s; the outlet valve {:.3} l/s",
        steady.pipe_flows[loop_pipe.index()].value() * 1e3,
        steady.pipe_flows[5].value() * 1e3
    );
    check(
        "energy equation satisfied (< 1e-9 m)",
        steady.max_energy_residual.value() < 1e-9,
    );
    check(
        "continuity satisfied (< 1e-12 m^3/s)",
        steady.max_continuity_residual.value() < 1e-12,
    );

    let probes = [
        Probe::Node(a),
        Probe::Node(b),
        Probe::Node(c),
        Probe::Node(v),
        Probe::Pipe {
            pipe: loop_pipe,
            fraction: 0.5,
        },
    ];
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(10.0),
            &probes,
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 4,
            },
        )
        .unwrap();
    let t = &run.trace;
    let (mut head_drift, mut flow_drift) = (0.0_f64, 0.0_f64);
    for i in 0..probes.len() {
        let (h0, q0) = (t.head(i)[0].value(), t.flow(i)[0].value());
        for k in 0..t.len() {
            head_drift = head_drift.max((t.head(i)[k].value() - h0).abs());
            flow_drift = flow_drift.max((t.flow(i)[k].value() - q0).abs());
        }
    }
    println!(
        "Transient run from that state, nothing disturbed, {} samples:",
        t.len()
    );
    println!("  largest head drift {head_drift:.1e} m, flow drift {flow_drift:.1e} m^3/s");
    check("no fake transient (head drift < 1e-6 m)", head_drift < 1e-6);
    check("flows hold (drift < 1e-9 m^3/s)", flow_drift < 1e-9);
}
