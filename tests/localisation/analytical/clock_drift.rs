//! Clocks that drift, repaired in software (frontier F9, second step).
//!
//! Underground - in a mine - no satellite reaches a logger: its clock is a
//! bare crystal that gains or loses a few parts per million (ppm). On the
//! estate network, S1 keeps GNSS time (the reference). S2 reads 40 ms late
//! at 06:00 and gains 20 ppm (1.7 s a day); S3 reads 25 ms early and loses
//! 35 ppm. Hydrants are opened on purpose at known places at 06:00, 10:00,
//! 14:00 and 20:00. A burst then happens at 23:00, after the last of them:
//! the repair must EXTRAPOLATE, and must say how unsure that makes it.

use jk_hydraulics::NodeId;
use jk_kernel::Time;
use jk_localisation::clocks::{correct, correct_drifting};
use jk_localisation::{
    Arrival, CalibrationEvent, LocaliseOptions, estimate_drift, estimate_offsets, localise,
};
use jk_tests::sensor::Rng;
use jk_tests::{check, check_close};

use crate::{A, arrivals_on_branch, estate};

/// 2026-09-29 06:00:00 UTC in seconds since 1970 (clock time, as loggers
/// stamp it).
const SIX: f64 = 1_790_661_600.0;
const HOUR: f64 = 3600.0;
/// Offset at 06:00 [s] and rate [s/s] of S1, S2, S3.
const CLOCKS: [(f64, f64); 3] = [(0.0, 0.0), (0.040, 20e-6), (-0.025, -35e-6)];
/// Calibration hydrants: hours after 06:00, and distances [m] from the
/// source to S1, S2, S3 (tee, S2, S3, tee).
const HYDRANTS: [(f64, [f64; 3]); 4] = [
    (0.0, [300.0, 500.0, 600.0]),
    (4.0, [800.0, 0.0, 1100.0]),
    (8.0, [900.0, 1100.0, 0.0]),
    (14.0, [300.0, 500.0, 600.0]),
];

/// What sensor `s`'s clock reads when the true time is `t`.
fn reading(s: usize, t: f64) -> f64 {
    let (offset, rate) = CLOCKS[s];
    t + offset + rate * (t - SIX)
}

fn events(
    sources: [NodeId; 4],
    sensors: [NodeId; 3],
    noise: f64,
    rng: &mut Rng,
) -> Vec<CalibrationEvent> {
    HYDRANTS
        .iter()
        .zip(sources)
        .map(|(&(hours, distances), source)| CalibrationEvent {
            source,
            arrivals: sensors
                .iter()
                .enumerate()
                .map(|(s, &node)| {
                    let t = SIX + hours * HOUR + distances[s] / A;
                    (node, Time::new(reading(s, t) + noise * rng.normal()))
                })
                .collect(),
        })
        .collect()
}

#[test]
fn f9_drift_rates_and_offsets_are_recovered_from_known_events() {
    let (net, _) = estate(None);
    let [s1, s2, s3] = net.sensors;
    let sources = [net.tee, s2, s3, net.tee];
    let mut rng = Rng::new(8);
    let late = SIX + 17.0 * HOUR; // 23:00

    println!("Noise-free picks, four known events between 06:00 and 20:00:");
    let exact = estimate_drift(
        &net.net,
        &events(sources, net.sensors, 0.0, &mut rng),
        s1,
        Time::new(0.2e-3),
    )
    .unwrap();
    for (s, est) in exact.iter().enumerate().skip(1) {
        // The rate is per second of the logger's OWN clock: a clock gaining
        // r per real second gains r / (1 + r) per second it reads. Clock
        // time (1.79e9 s) is stored to 0.24 us, so over the 14 hours of
        // events a rate is known to about 1e-11: 1e-6 of 20 ppm.
        let (_, rate) = CLOCKS[s];
        check_close(
            &format!("  S{} drift [ppm of its own seconds]", s + 1),
            est.rate * 1e6,
            rate / (1.0 + rate) * 1e6,
            1e-6,
        );
        // Corrections take the logger's own reading of the moment.
        let read = Time::new(reading(s, late));
        check_close(
            &format!("  S{} offset at 23:00 [ms]", s + 1),
            est.offset_at(read).value() * 1e3,
            (reading(s, late) - late) * 1e3,
            1e-6,
        );
    }

    println!("Picks with 0.2 ms noise:");
    let noisy = estimate_drift(
        &net.net,
        &events(sources, net.sensors, 0.2e-3, &mut rng),
        s1,
        Time::new(0.2e-3),
    )
    .unwrap();
    for (s, est) in noisy.iter().enumerate().skip(1) {
        let truth = reading(s, late) - late;
        let read = Time::new(reading(s, late));
        let at_epoch = est.uncertainty_at(est.epoch).value();
        let at_late = est.uncertainty_at(read).value();
        println!(
            "  S{}: drift {:+.3} ppm (true {:+.0}); offset at 23:00 {:+.3} ms (true {:+.3}), \
             uncertainty {:.3} ms (at the events' middle {:.3} ms)",
            s + 1,
            est.rate * 1e6,
            CLOCKS[s].1 * 1e6,
            est.offset_at(read).value() * 1e3,
            truth * 1e3,
            at_late * 1e3,
            at_epoch * 1e3
        );
        check(
            "    offset at 23:00 within 3 reported sigma",
            (est.offset_at(read).value() - truth).abs() <= 3.0 * at_late,
        );
        check(
            "    less sure when extrapolating than at the events",
            at_late > at_epoch,
        );
    }
}

#[test]
fn f9_a_burst_after_the_last_calibration_is_found_only_with_drift_repaired() {
    let (net, _) = estate(None);
    let [s1, s2, s3] = net.sensors;
    let sources = [net.tee, s2, s3, net.tee];
    let mut rng = Rng::new(12);
    let calibration = events(sources, net.sensors, 0.1e-3, &mut rng);
    let drift = estimate_drift(&net.net, &calibration, s1, Time::new(0.1e-3)).unwrap();
    let offsets = estimate_offsets(&net.net, &calibration, s1, Time::new(0.1e-3)).unwrap();

    // The burst: 200 m along T-S3 at 23:00, true time; each logger stamps
    // it on its own drifting clock.
    let burst = SIX + 17.0 * HOUR;
    let truth = arrivals_on_branch(200.0, burst);
    let raw: Vec<(NodeId, Time)> = net
        .sensors
        .iter()
        .enumerate()
        .map(|(s, &node)| (node, Time::new(reading(s, truth[s]))))
        .collect();
    let pick = Time::new(0.1e-3);
    let options = LocaliseOptions {
        wave_speed_uncertainty: 0.02,
        ..LocaliseOptions::default()
    };
    let error = |arrivals: &[Arrival]| {
        let l = localise(&net.net, arrivals, &options).unwrap();
        let on_branch = l.most_likely.pipe == net.pipes[3];
        let off = if on_branch {
            (l.most_likely.distance.value() - 200.0).abs()
        } else {
            f64::INFINITY
        };
        (off, l.credible_length.value(), l.misfit)
    };
    let as_read: Vec<Arrival> = raw
        .iter()
        .map(|&(node, time)| Arrival {
            node,
            time,
            uncertainty: pick,
        })
        .collect();
    let fixed: Vec<Arrival> = raw
        .iter()
        .map(|&(node, time)| correct(node, time, pick, &offsets, Time::new(0.05)))
        .collect();
    let drifting: Vec<Arrival> = raw
        .iter()
        .map(|&(node, time)| correct_drifting(node, time, pick, &drift, Time::new(0.05)))
        .collect();
    println!("Burst 200 m along T-S3 at 23:00 (clocks: S2 +20 ppm, S3 -35 ppm, no GNSS):");
    for (what, arrivals) in [
        ("clocks as read", &as_read),
        ("one fixed offset per clock", &fixed),
        ("offset and drift per clock", &drifting),
    ] {
        let (off, region, misfit) = error(arrivals);
        if off.is_finite() {
            println!(
                "  {what:<28} {off:>8.2} m off, 90% stretch {region:>7.1} m, misfit {misfit:>8.2}"
            );
        } else {
            println!(
                "  {what:<28} wrong pipe,     90% stretch {region:>7.1} m, misfit {misfit:>8.2}"
            );
        }
    }
    let (off, region, _) = error(&drifting);
    check("drift repaired: within 1 m of the burst", off < 1.0);
    check("drift repaired: 90% stretch under 30 m", region < 30.0);
    let (off_fixed, _, _) = error(&fixed);
    check(
        "a fixed offset alone misses by more than 50 m",
        off_fixed > 50.0,
    );
}

#[test]
fn f9_a_clock_seen_at_only_one_time_has_no_measurable_drift_and_is_refused() {
    let (net, _) = estate(None);
    let [s1, s2, _] = net.sensors;
    let sources = [net.tee, s2, net.sensors[2], net.tee];
    let mut rng = Rng::new(4);
    let mut calibration = events(sources, net.sensors, 0.0, &mut rng);
    // S2 is only switched on for the 10:00 event.
    for (e, event) in calibration.iter_mut().enumerate() {
        if e != 1 {
            event.arrivals.retain(|&(node, _)| node != s2);
        }
    }
    let err = estimate_drift(&net.net, &calibration, s1, Time::new(0.1e-3)).unwrap_err();
    println!("  refused: {err}");
    check(
        "the refusal names S2's node",
        err.to_string().contains(&format!("node {}", s2.index())),
    );
}
