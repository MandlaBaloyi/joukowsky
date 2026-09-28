//! Two independent solvers must agree.
//!
//! The reservoir -> pipe -> valve system is solved twice: by the dedicated
//! single-pipe solver (already proven against Joukowsky, V-A1) and by the
//! general network solver. Then the same pipe is split in two at a junction
//! - a junction between identical pipes must be invisible to the wave.

use jk_hydraulics::{InitialCondition, Network, Node, Probe, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{V0, check, pipe, standard_system};

use super::{options_for_dt_005, reference_valve_heads};

/// Valve-head history from the network solver, with the 1000 m pipe split
/// into the given `lengths` joined by plain junctions.
fn network_valve_heads(
    lengths: &[f64],
    friction: f64,
    closure: ValveClosure,
    duration: f64,
) -> Vec<f64> {
    let standard = standard_system(friction, closure, 20);
    let q0 = standard.initial_flow;
    let mut net = Network::new(STANDARD_GRAVITY);
    let mut prev = net.add_node(Node::Reservoir {
        head: Head::new(100.0),
    });
    for (i, &l) in lengths.iter().enumerate() {
        let next = if i + 1 == lengths.len() {
            net.add_node(Node::Valve {
                downstream_head: Head::new(50.0),
                closure,
            })
        } else {
            net.add_node(Node::Junction { demand: Flow::ZERO })
        };
        net.add_pipe(prev, next, pipe(l, 0.3, 1000.0, friction))
            .unwrap();
        prev = next;
    }
    let shortest = lengths.iter().copied().fold(f64::INFINITY, f64::min);
    let run = net
        .simulate(
            &InitialCondition::PipeFlows(vec![q0; lengths.len()]),
            Time::new(duration),
            &[Probe::Node(prev)],
            &options_for_dt_005(shortest),
        )
        .unwrap();
    run.trace.head(0).iter().map(|h| h.value()).collect()
}

fn largest_difference(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "histories must have the same length");
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

#[test]
fn network_solver_reproduces_the_single_pipe_solver() {
    println!("Reservoir -> 1000 m -> valve at {V0} m/s, solved by both solvers:");
    for (name, friction, closure) in [
        (
            "instant closure, no friction",
            0.0,
            ValveClosure::Instantaneous,
        ),
        (
            "instant closure, f = 0.02",
            0.02,
            ValveClosure::Instantaneous,
        ),
        (
            "slow closure, f = 0.02",
            0.02,
            ValveClosure::linear(Time::new(6.0)),
        ),
    ] {
        let reference = reference_valve_heads(friction, closure, 12.0);
        let network = network_valve_heads(&[1000.0], friction, closure, 12.0);
        let diff = largest_difference(&reference, &network);
        println!(
            "  {name:<32} largest difference {diff:.1e} m over {} samples",
            reference.len()
        );
        check("  solvers agree within 1e-9 m", diff < 1e-9);
    }
}

#[test]
fn splitting_a_pipe_at_a_junction_changes_nothing() {
    println!("The same 1000 m pipe, cut into pieces joined by junctions:");
    let reference = reference_valve_heads(0.02, ValveClosure::Instantaneous, 12.0);
    for lengths in [
        vec![400.0, 600.0],
        vec![100.0, 250.0, 650.0],
        vec![500.0, 500.0],
    ] {
        let network = network_valve_heads(&lengths, 0.02, ValveClosure::Instantaneous, 12.0);
        let diff = largest_difference(&reference, &network);
        println!("  pieces {lengths:?}: largest difference {diff:.1e} m");
        check(
            "  junctions between identical pipes are invisible",
            diff < 1e-9,
        );
    }
}
