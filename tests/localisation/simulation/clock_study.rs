//! Study V-S1, part 2: how clock quality limits location - and repairing it.
//!
//! Part 1 (tests/detection) measured location error with perfect clocks.
//! Here each sensor's clock is off by a random amount of standard deviation
//! `sigma_c`. With two sensors, x = (L + a (t1 - t2)) / 2, so the clock
//! disagreement c1 - c2 (standard deviation sqrt(2) `sigma_c`) moves the
//! burst by a `sigma_c` / sqrt(2) - from 7 cm for a GNSS clock to 35 m for NTP.

use jk_hydraulics::{Network, Node};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_localisation::clocks::correct;
use jk_localisation::{Arrival, CalibrationEvent, LocaliseOptions, estimate_offsets, localise};
use jk_tests::sensor::Rng;
use jk_tests::{check, pipe};

use crate::{A, arrivals_on_branch, estate};

/// R1 --300-- S1 --2000-- S2 --300-- R2; returns the network and sensors.
fn straight_main() -> (Network, [jk_hydraulics::NodeId; 2]) {
    let main = |l| pipe(l, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r1 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(j());
    let s2 = net.add_node(j());
    let r2 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    net.add_pipe(r1, s1, main(300.0)).unwrap();
    net.add_pipe(s1, s2, main(2000.0)).unwrap();
    net.add_pipe(s2, r2, main(300.0)).unwrap();
    (net, [s1, s2])
}

#[test]
fn vs1_location_error_grows_with_clock_error_as_theory_says() {
    let (net, [s1, s2]) = straight_main();
    let pick = 0.15e-3; // what the detector achieves at 128 S/s
    let mut rng = Rng::new(11);
    println!("Burst 700 m from S1; clocks off by sigma_c each; 100 trials per row:");
    println!(
        "  clock           sigma_c    RMS error   theory a sigma_c/sqrt2   90% region holds truth"
    );
    for (name, sigma_c) in [
        ("GNSS", 1e-4),
        ("good NTP", 1e-3),
        ("NTP cellular", 1e-2),
        ("poor NTP", 5e-2),
    ] {
        let (mut sum_sq, mut inside) = (0.0, 0_u32);
        let trials = 100_u32;
        for _ in 0..trials {
            let (c1, c2) = (sigma_c * rng.normal(), sigma_c * rng.normal());
            let declared = Time::new(f64::hypot(pick, sigma_c));
            let arrivals = [
                Arrival {
                    node: s1,
                    time: Time::new(2.7 + c1 + pick * rng.normal()),
                    uncertainty: declared,
                },
                Arrival {
                    node: s2,
                    time: Time::new(3.3 + c2 + pick * rng.normal()),
                    uncertainty: declared,
                },
            ];
            let loc = localise(&net, &arrivals, &LocaliseOptions::default()).unwrap();
            let err = loc.most_likely.distance.value() - 700.0;
            sum_sq += err * err;
            if loc
                .credible_region
                .iter()
                .any(|s| s.from.value() <= 700.0 && 700.0 <= s.to.value())
            {
                inside += 1;
            }
        }
        let rms = (sum_sq / f64::from(trials)).sqrt();
        let theory = A * f64::hypot(pick, sigma_c) / std::f64::consts::SQRT_2;
        let coverage = f64::from(inside) / f64::from(trials);
        println!(
            "  {name:<14} {:>6.1} ms   {rms:>8.2} m   {theory:>12.2} m            {:>5.0}%",
            sigma_c * 1e3,
            coverage * 100.0
        );
        check(
            "  RMS within 30% of theory",
            (rms / theory - 1.0).abs() < 0.3,
        );
        check("  the 90% region stays honest (>= 80%)", coverage >= 0.8);
    }
    println!("  => metre-level location needs clocks good to ~1 ms: GNSS, or repaired.");
}

#[test]
fn f9_software_clock_repair_restores_metre_level_location() {
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let offsets = [0.0, 0.023, -0.041]; // S1 GNSS; S2, S3 NTP over cellular
    let pick = 0.2e-3;
    let ntp = 0.04; // what an unrepaired NTP clock must be assumed to be off by
    let mut rng = Rng::new(21);
    let raw = |t: [f64; 3], rng: &mut Rng| -> [f64; 3] {
        std::array::from_fn(|i| t[i] + offsets[i] + pick * rng.normal())
    };

    // A burst 200 m along branch T-S3, on the sensors' own (wrong) clocks.
    let burst = raw(arrivals_on_branch(200.0, 50.0), &mut rng);

    println!("Burst 200 m along branch T-S3; S2 reads 23 ms late, S3 41 ms early.");
    let unrepaired: Vec<Arrival> = e
        .sensors
        .iter()
        .zip(burst)
        .enumerate()
        .map(|(i, (&node, t))| Arrival {
            node,
            time: Time::new(t),
            uncertainty: Time::new(if i == 0 { pick } else { f64::hypot(pick, ntp) }),
        })
        .collect();
    let before = localise(&e.net, &unrepaired, &LocaliseOptions::default()).unwrap();
    println!(
        "  WITHOUT repair: most likely pipe {} at {:.1} m; 90% region {:.0} m of pipe",
        before.most_likely.pipe.index(),
        before.most_likely.distance.value(),
        before.credible_length.value()
    );

    // Calibration: hydrants opened on purpose at the tee, S2 and S3.
    let calib: Vec<CalibrationEvent> = [
        (e.tee, [300.0, 500.0, 600.0]),
        (s2, [800.0, 0.0, 1100.0]),
        (s3, [900.0, 1100.0, 0.0]),
    ]
    .iter()
    .enumerate()
    .map(|(k, &(source, d))| {
        let t0 = 100.0 + 60.0 * f64::from(u8::try_from(k).unwrap());
        let t = raw(std::array::from_fn(|i| t0 + d[i] / A), &mut rng);
        CalibrationEvent {
            source,
            arrivals: e.sensors.iter().copied().zip(t.map(Time::new)).collect(),
        }
    })
    .collect();
    let clocks = estimate_offsets(&e.net, &calib, s1, Time::new(pick)).unwrap();
    for c in &clocks {
        println!(
            "  estimated offset of node {}: {:+.2} ms (+/- {:.2} ms)",
            c.node.index(),
            c.offset.value() * 1e3,
            c.uncertainty.value() * 1e3
        );
    }
    let repaired: Vec<Arrival> = e
        .sensors
        .iter()
        .zip(burst)
        .map(|(&node, t)| correct(node, Time::new(t), Time::new(pick), &clocks, Time::new(ntp)))
        .collect();
    let after = localise(&e.net, &repaired, &LocaliseOptions::default()).unwrap();
    let error = (after.most_likely.distance.value() - 200.0).abs();
    println!(
        "  WITH repair: pipe {} at {:.2} m (true 200 m); 90% region {:.1} m of pipe",
        after.most_likely.pipe.index(),
        after.most_likely.distance.value(),
        after.credible_length.value()
    );
    check(
        "repaired: the right branch",
        after.most_likely.pipe == e.pipes[3],
    );
    check("repaired: within 1 m", error < 1.0);
    check(
        "repair shrinks the 90% region at least tenfold",
        after.credible_length.value() * 10.0 <= before.credible_length.value(),
    );
}
