//! Clock offsets recovered from known-source events (software clock repair).
//!
//! On the estate network, S1 has a GNSS-disciplined clock (the reference);
//! S2 and S3 are NTP-synchronised over cellular and read 23 ms late and
//! 41 ms early. Three events with KNOWN sources - an operator opening a
//! hydrant at the tee, at S2 and at S3 - are seen by all three sensors.

use jk_hydraulics::NodeId;
use jk_kernel::Time;
use jk_localisation::{CalibrationEvent, estimate_offsets};
use jk_tests::sensor::Rng;
use jk_tests::{check, check_close};

use crate::{A, estate};

/// Distances [m] from each known source (tee, S2, S3) to S1, S2, S3.
const DISTANCES: [[f64; 3]; 3] = [
    [300.0, 500.0, 600.0], // hydrant at the tee
    [800.0, 0.0, 1100.0],  // hydrant at S2
    [900.0, 1100.0, 0.0],  // hydrant at S3
];
const OFFSETS: [f64; 3] = [0.0, 0.023, -0.041];

fn events(
    sources: [NodeId; 3],
    sensors: [NodeId; 3],
    noise: f64,
    rng: &mut Rng,
) -> Vec<CalibrationEvent> {
    sources
        .iter()
        .enumerate()
        .map(|(e, &source)| CalibrationEvent {
            source,
            arrivals: sensors
                .iter()
                .enumerate()
                .map(|(s, &node)| {
                    let t0 = 10.0 * (1.0 + f64::from(u8::try_from(e).unwrap()));
                    let t = t0 + DISTANCES[e][s] / A + OFFSETS[s] + noise * rng.normal();
                    (node, Time::new(t))
                })
                .collect(),
        })
        .collect()
}

#[test]
fn offsets_are_recovered_from_three_known_events() {
    let (net, _) = estate(None);
    let [s1, s2, s3] = net.sensors;
    let sources = [net.tee, s2, s3];
    let mut rng = Rng::new(3);

    println!("Noise-free picks:");
    let exact = estimate_offsets(
        &net.net,
        &events(sources, net.sensors, 0.0, &mut rng),
        s1,
        Time::new(0.2e-3),
    )
    .unwrap();
    for (est, truth) in exact.iter().zip(OFFSETS) {
        check_close(
            &format!("  offset of node {} [ms]", est.node.index()),
            est.offset.value() * 1e3,
            truth * 1e3,
            1e-9,
        );
    }

    println!("Picks with 0.2 ms noise:");
    let noisy = estimate_offsets(
        &net.net,
        &events(sources, net.sensors, 0.2e-3, &mut rng),
        s1,
        Time::new(0.2e-3),
    )
    .unwrap();
    for (est, truth) in noisy.iter().zip(OFFSETS).skip(1) {
        let err = est.offset.value() - truth;
        println!(
            "  node {}: {:+.3} ms (true {:+.1} ms), uncertainty {:.3} ms",
            est.node.index(),
            est.offset.value() * 1e3,
            truth * 1e3,
            est.uncertainty.value() * 1e3
        );
        check(
            "  within 3 reported sigma",
            err.abs() <= 3.0 * est.uncertainty.value(),
        );
    }
}

#[test]
fn a_clock_that_never_meets_the_others_is_refused_not_guessed() {
    let (net, _) = estate(None);
    let [s1, s2, s3] = net.sensors;
    // Plenty of data overall - S1 and S3 share three events - but S2 only
    // ever sees events on its own: its clock and those events' times cannot
    // be told apart, however many there are.
    let at = |node, t| (node, Time::new(t));
    let alone = vec![
        CalibrationEvent {
            source: net.tee,
            arrivals: vec![at(s1, 10.3), at(s3, 10.6)],
        },
        CalibrationEvent {
            source: net.tee,
            arrivals: vec![at(s1, 20.3), at(s3, 20.6)],
        },
        CalibrationEvent {
            source: net.tee,
            arrivals: vec![at(s1, 30.3), at(s3, 30.6)],
        },
        CalibrationEvent {
            source: s2,
            arrivals: vec![at(s2, 40.0)],
        },
        CalibrationEvent {
            source: s2,
            arrivals: vec![at(s2, 50.0)],
        },
    ];
    let result = estimate_offsets(&net.net, &alone, s1, Time::new(0.2e-3));
    println!(
        "S2 only ever sees an event alone: {}",
        match &result {
            Ok(_) => "an offset was invented (WRONG)".to_string(),
            Err(e) => format!("refused - {e}"),
        }
    );
    check("the engine refuses", result.is_err());
}
