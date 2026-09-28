//! Study V-S2, first check: does the junction-by-junction front model
//! agree with the full simulator?
//!
//! The placement advisor decides whether a burst is FELT at a logger from a
//! quick model: the drop at the hole, (B / 2) Q, times the transmission
//! coefficient of every junction on the fastest path. Here the method of
//! characteristics simulates the real thing - an 8 mm hole (about 1 l/s)
//! opening suddenly 200 m along T-S3 on the resting estate - and each
//! logger's first front is read off the trace, after the front and before
//! the first reflection:
//!
//! ```text
//!   logger   first front    next wave     read at (after the burst)
//!   hole     0 s            0.4 s (T)     0.1 s
//!   S3       0.4 s          0.8 s         0.5 s
//!   S1       0.5 s          1.3 s         0.6 s
//!   S2       0.7 s          1.5 s         0.8 s
//! ```
//!
//! The hole's flow depends on the head it sees, Q = k sqrt(H - h0), so the
//! model's h0 is found self-consistently: h0 = (B / 2) k sqrt(60 - h0).

use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Burst, BurstOpening, GridOptions, InitialCondition, Probe};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_localisation::Point;
use jk_localisation::placement::first_fronts;
use jk_tests::check_close;

use crate::{A, estate};

#[test]
fn the_quick_front_model_matches_the_full_simulation() {
    let hole = leak::coefficient(0.6, Area::of_circle(Length::new(0.008)), STANDARD_GRAVITY);
    let burst_at = 1.0;
    let (sim, burst_node) = estate(Some(200.0));
    let burst_node = burst_node.expect("split");
    let mut net = sim.net;
    net.add_burst(Burst {
        node: burst_node,
        coefficient: hole,
        outside_head: Head::ZERO,
        start: Time::new(burst_at),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let [s1, s2, s3] = sim.sensors;
    let probes = [burst_node, s3, s1, s2].map(Probe::Node);
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(2.0),
            &probes,
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 200,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let head_at = |probe: usize, t: f64| run.trace.head(probe)[(t / dt).round() as usize].value();

    // The model, on the plain (unsplit) estate map.
    let b = A / (STANDARD_GRAVITY.value() * Area::of_circle(Length::new(0.15)).value());
    let mut h0: f64 = 0.0;
    for _ in 0..50 {
        h0 = 0.5 * b * hole * (60.0 - h0).sqrt();
    }
    let flow = hole * (60.0 - h0).sqrt();
    let (e, _) = estate(None);
    let model = first_fronts(
        &e.net,
        &e.sensors,
        Point {
            pipe: e.pipes[3],
            distance: Length::new(200.0),
        },
        Flow::new(flow),
    )
    .unwrap();
    println!(
        "An 8 mm hole lets out {:.3} l/s; the drop at the hole is {h0:.3} m.",
        flow * 1000.0
    );
    println!("First front, full simulation vs the quick model:");
    let felt = |probe: usize, after: f64| {
        head_at(probe, burst_at - 0.05) - head_at(probe, burst_at + after)
    };
    check_close("at the hole", felt(0, 0.1), h0, 0.002);
    check_close("S3 (dead end)", felt(1, 0.5), model[2].value(), 0.002);
    check_close(
        "S1 (through the tee)",
        felt(2, 0.6),
        model[0].value(),
        0.002,
    );
    check_close(
        "S2 (tee, then dead end)",
        felt(3, 0.8),
        model[1].value(),
        0.002,
    );
}
