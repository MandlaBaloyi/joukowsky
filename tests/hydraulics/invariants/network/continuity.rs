//! Water is neither created nor destroyed at a junction (principle P1: a
//! result that violates conservation of mass is a bug).
//!
//! A junction with a consumer demand feeds two branches whose valves do
//! different things - one shuts slowly, one stays open - so the flows swing
//! for the whole run. At every one of the samples, what arrives must equal
//! what leaves.
//!
//! ```text
//!                           /-- 400 m --> valve 1 (shuts over 3 s)
//!   R --- 600 m --- J (demand 5 l/s)
//!                           \-- 250 m --> valve 2 (stays open)
//! ```

use jk_hydraulics::{GridOptions, InitialCondition, Network, Node, PipeId, Probe, ValveClosure};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::{check, pipe};

#[test]
fn flows_balance_at_the_junction_at_every_instant() {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(90.0),
    });
    let j = net.add_node(Node::Junction {
        demand: Flow::new(0.005),
    });
    let v1 = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::linear(Time::new(3.0)),
    });
    let v2 = net.add_node(Node::Valve {
        downstream_head: Head::ZERO,
        closure: ValveClosure::Open,
    });
    let p0 = net.add_pipe(r, j, pipe(600.0, 0.25, 1050.0, 0.02)).unwrap();
    let p1 = net.add_pipe(j, v1, pipe(400.0, 0.2, 1050.0, 0.02)).unwrap();
    let p2 = net
        .add_pipe(j, v2, pipe(250.0, 0.15, 1050.0, 0.02))
        .unwrap();

    let flows = vec![Flow::new(0.040), Flow::new(0.020), Flow::new(0.015)];
    let at = |pipe: PipeId, fraction: f64| Probe::Pipe { pipe, fraction };
    let run = net
        .simulate(
            &InitialCondition::PipeFlows(flows),
            Time::new(12.0),
            &[at(p0, 1.0), at(p1, 0.0), at(p2, 0.0), Probe::Node(j)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 5,
            },
        )
        .unwrap();
    let t = &run.trace;
    let mut worst = 0.0_f64;
    for k in 0..t.len() {
        let arriving = t.flow(0)[k].value();
        let leaving = t.flow(1)[k].value() + t.flow(2)[k].value() + t.flow(3)[k].value();
        worst = worst.max((arriving - leaving).abs());
    }
    println!("  samples checked: {}", t.len());
    println!(
        "  flow into branch 1 went from {:.2} to {:.2} l/s as its valve shut",
        t.flow(1)[0].value() * 1e3,
        t.flow(1)[t.len() - 1].value() * 1e3
    );
    println!("  largest imbalance at the junction: {worst:.1e} m^3/s");
    check(
        "mass balances to rounding error at every instant",
        worst < 1e-12,
    );
}
