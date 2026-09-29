//! The answer must not depend on where the clock starts.
//!
//! Simulations start at t = 0; real loggers stamp clock time - seconds
//! since 1970, about 1.79e9 today. Found the hard way (a burst read from
//! logger files came out with a misfit of exactly 2^18 and a 1.2 km search
//! stretch): scored with raw clock times, the squared residuals lost all
//! their significant digits. Both the locator and the clock repair now
//! measure times from the earliest arrival. These tests keep it that way.

use jk_hydraulics::NodeId;
use jk_kernel::Time;
use jk_localisation::{Arrival, CalibrationEvent, LocaliseOptions, estimate_offsets, localise};
use jk_tests::{check, check_close};

use crate::{arrivals_on_branch, estate};

/// 2026-09-29 10:15:00 UTC in seconds since 1970.
const CLOCK: f64 = 1_790_676_900.0;

#[test]
fn locating_with_clock_time_gives_the_same_answer_as_from_zero() {
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let options = LocaliseOptions {
        wave_speed_uncertainty: 0.05,
        ..LocaliseOptions::default()
    };
    let locate = |t0: f64| {
        let t = arrivals_on_branch(200.0, t0);
        let arrivals: Vec<Arrival> = [s1, s2, s3]
            .iter()
            .zip(t)
            .map(|(&node, time)| Arrival {
                node,
                time: Time::new(time),
                uncertainty: Time::new(0.5e-3),
            })
            .collect();
        localise(&e.net, &arrivals, &options).unwrap()
    };
    let (zero, clock) = (locate(2.0), locate(CLOCK + 2.0));
    println!("Burst 200 m along T-S3, located with times near 0 and near {CLOCK:.0} s:");
    check_close(
        "most likely point [m]",
        clock.most_likely.distance.value(),
        zero.most_likely.distance.value(),
        1e-6,
    );
    check("same pipe", clock.most_likely.pipe == zero.most_likely.pipe);
    check_close(
        "90% search stretch [m]",
        clock.credible_length.value(),
        zero.credible_length.value(),
        1e-6,
    );
    check_close(
        "emission time, clock minus zero [s]",
        clock.emission.value() - zero.emission.value(),
        CLOCK,
        1e-12,
    );
    check(
        "misfit stays small (arrivals fit one source)",
        clock.misfit < 1.0,
    );
}

#[test]
fn clock_repair_with_clock_time_gives_the_same_offsets() {
    // Two known events at the tee and at S3; S2's clock runs 3 ms late.
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let events = |base: f64| -> Vec<CalibrationEvent> {
        let at = |source: NodeId, t: [f64; 3]| CalibrationEvent {
            source,
            arrivals: vec![
                (s1, Time::new(base + t[0])),
                (s2, Time::new(base + t[1] + 0.003)),
                (s3, Time::new(base + t[2])),
            ],
        };
        // Travel times at 1000 m/s: tee -> S1 0.3, S2 0.5, S3 0.6 s;
        // S3 -> S1 0.9, S2 1.1, S3 0 s.
        vec![
            at(e.tee, [10.3, 10.5, 10.6]),
            at(s3, [20.9, 21.1, 20.0]),
            at(e.tee, [30.3, 30.5, 30.6]),
        ]
    };
    let sigma = Time::new(0.1e-3);
    let zero = estimate_offsets(&e.net, &events(0.0), s1, sigma).unwrap();
    let clock = estimate_offsets(&e.net, &events(CLOCK), s1, sigma).unwrap();
    // A clock time near 1.79e9 s is itself stored to about 0.24 us (the
    // spacing of 64-bit numbers there), so the offsets can agree only to
    // that: 1 us is the test - finer than a GNSS clock (about 1 us).
    for (a, b) in zero.iter().zip(&clock) {
        let diff = (b.offset.value() - a.offset.value()).abs();
        println!(
            "  offset of node {}: {:+.6} ms from zero, {:+.6} ms from clock time",
            a.node.index(),
            a.offset.value() * 1000.0,
            b.offset.value() * 1000.0
        );
        check("  ... the same to 1 microsecond", diff < 1e-6);
    }
    let s2_offset = clock.iter().find(|c| c.node == s2).unwrap().offset.value();
    check(
        "S2's clock found 3 ms late from clock time (to 1 us)",
        (s2_offset - 0.003).abs() < 1e-6,
    );
}
