//! The engine's core idea, end to end: a burst's own pressure wave locates it.
//!
//! `docs/04_detection_and_localisation/burst_localisation.txt` section 1.
//! A main runs between two reservoirs, with sensors S1 and S2 2000 m apart
//! and a burst 700 m from S1. Everything is at rest at 60 m head. The burst
//! sends a negative step both ways at a = 1000 m/s; it reaches S1 after
//! 0.7 s and S2 after 1.3 s. From the two arrival times alone:
//!
//! ```text
//!   x = (L + a (t1 - t2)) / 2        distance of the burst from S1
//! ```
//!
//! and from the size of the step, how much water it is losing.
//!
//! ```text
//!   R1 --300 m-- S1 ------700 m------ BURST -----------1300 m----------- S2 --300 m-- R2
//! ```

use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, Probe, joukowsky,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::{check, check_close, pipe};

const A: f64 = 1000.0;
const H0: f64 = 60.0;

#[test]
fn two_sensors_locate_and_size_a_burst() {
    let main = |length| pipe(length, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let r1 = net.add_node(Node::Reservoir {
        head: Head::new(H0),
    });
    let s1 = net.add_node(Node::Junction { demand: Flow::ZERO });
    let b = net.add_node(Node::Junction { demand: Flow::ZERO });
    let s2 = net.add_node(Node::Junction { demand: Flow::ZERO });
    let r2 = net.add_node(Node::Reservoir {
        head: Head::new(H0),
    });
    net.add_pipe(r1, s1, main(300.0)).unwrap();
    net.add_pipe(s1, b, main(700.0)).unwrap();
    net.add_pipe(b, s2, main(1300.0)).unwrap();
    net.add_pipe(s2, r2, main(300.0)).unwrap();
    let hole = Area::of_circle(Length::new(0.010));
    net.add_burst(Burst {
        node: b,
        coefficient: leak::coefficient(0.6, hole, STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(1.0),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();

    let options = GridOptions {
        max_wave_speed_adjustment: 0.01,
        min_reaches: 30, // 300 m in 30 reaches: dt = 0.01 s, dx = 10 m
    };
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(H0),
            },
            Time::new(4.0),
            &[Probe::Node(s1), Probe::Node(s2), Probe::Node(b)],
            &options,
        )
        .unwrap();
    let trace = &run.trace;
    let dt = trace.time_step().value();
    let dx = A * dt;
    println!(
        "Grid: dt = {dt} s, dx = {dx} m, largest wave-speed adjustment {:.1e}",
        run.grid.max_adjustment()
    );

    // Onset: first sample more than 1 m below the resting head.
    let onset = |probe: usize| {
        let k = trace
            .head(probe)
            .iter()
            .position(|h| h.value() < H0 - 1.0)
            .expect("the burst wave must reach every sensor");
        (k, trace.time(k).value())
    };
    let (k_burst, t_burst) = onset(2);
    let (k1, t1) = onset(0);
    let (_, t2) = onset(1);
    println!("Arrival times:");
    println!("  burst node  {t_burst:.3} s");
    println!("  sensor S1   {t1:.3} s   (expected burst + 0.7 s)");
    println!("  sensor S2   {t2:.3} s   (expected burst + 1.3 s)");

    println!("Location from the arrival times alone:");
    let l = 2000.0;
    #[allow(clippy::manual_midpoint)] // written exactly as in the docs
    let x = (l + A * (t1 - t2)) / 2.0;
    check_close("burst distance from S1 [m]", x, 700.0, dx / 700.0);
    let t0 = t1 - x / A;
    check_close("burst time recovered [s]", t0, t_burst, dt / t_burst);

    println!("Size from the step at S1:");
    let step = trace.head(0)[k1 + 2].value() - H0;
    let lost = trace.flow(2)[k_burst + 2].value();
    println!(
        "  step seen at S1           {step:.3} m ({:.2} bar)",
        step / 10.2
    );
    println!("  true burst outflow        {:.3} l/s", lost * 1e3);
    let sized = joukowsky::burst_flow_from_head_drop(
        Head::new(step),
        jk_kernel::WaveSpeed::new(A),
        Area::of_circle(Length::new(0.15)),
        STANDARD_GRAVITY,
    );
    check_close(
        "burst flow sized from the step [l/s]",
        sized.value() * 1e3,
        lost * 1e3,
        0.01,
    );
    check("S2 hears it later than S1 (closer to S1)", t2 > t1);
    println!(
        "  => estimated loss {:.0} m^3/day, located to within one {dx} m reach.",
        sized.value() * 86_400.0
    );
}
