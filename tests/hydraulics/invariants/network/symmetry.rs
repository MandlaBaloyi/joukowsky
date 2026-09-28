//! A mirror-symmetric network must give mirror-identical answers - to the
//! last bit, because the solver does exactly the same arithmetic on both
//! sides. Any asymmetry would mean the solver depends on the ORDER in which
//! pipes were entered, which would be a bug.
//!
//! ```text
//!                      /-- 300 m --> valve A (shuts instantly)
//!   R --- 500 m --- tee
//!                      \-- 300 m --> valve B (shuts instantly)
//! ```

use jk_hydraulics::{GridOptions, InitialCondition, Network, Node, Probe, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, pipe};

#[test]
fn mirror_branches_give_bit_identical_results() {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(80.0),
    });
    let tee = net.add_node(Node::Junction { demand: Flow::ZERO });
    let valve = || Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Instantaneous,
        flow_coefficient: None,
    };
    let va = net.add_node(valve());
    let vb = net.add_node(valve());
    net.add_pipe(r, tee, pipe(500.0, 0.2, 1100.0, 0.02))
        .unwrap();
    net.add_pipe(tee, va, pipe(300.0, 0.15, 1100.0, 0.02))
        .unwrap();
    net.add_pipe(tee, vb, pipe(300.0, 0.15, 1100.0, 0.02))
        .unwrap();

    let flows = vec![Flow::new(0.03), Flow::new(0.015), Flow::new(0.015)];
    let run = net
        .simulate(
            &InitialCondition::PipeFlows(flows),
            Time::new(10.0),
            &[Probe::Node(va), Probe::Node(vb)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 6,
            },
        )
        .unwrap();
    let (a, b) = (run.trace.head(0), run.trace.head(1));
    let mismatches = a
        .iter()
        .zip(b)
        .filter(|(x, y)| x.value().to_bits() != y.value().to_bits())
        .count();
    println!("  samples compared bit by bit: {}", a.len());
    println!(
        "  peak head at valve A {:.3} m, at valve B {:.3} m",
        run.trace.max_head(0).value(),
        run.trace.max_head(1).value()
    );
    check("every sample identical to the last bit", mismatches == 0);
}
