//! How precisely is the arrival of a front timed?
//!
//! Location error = a x timing error / 2, so this is the number that turns
//! into metres on the map. At 128 S/s one sample is 7.8 ms (~4 m at
//! a = 1000 m/s); the target is a small fraction of that.

use jk_tests::check;
use jk_tests::sensor::{Logger, Rng, sample};

use crate::detect;

const RATE: f64 = 128.0;

/// A sharp 4.5 m drop at `t_step`, simulated every 0.1 ms for 4 s.
fn truth(t_step: f64) -> (Vec<f64>, f64) {
    let dt = 1e-4;
    let v = (0..40_000)
        .map(|i| {
            if f64::from(i) * dt < t_step {
                60.0
            } else {
                55.5
            }
        })
        .collect();
    (v, dt)
}

#[test]
fn a_noise_free_front_is_timed_to_a_tiny_fraction_of_a_sample() {
    println!("Sharp 4.5 m drop, 128 S/s integrating logger, no noise, 25 clock phases:");
    let (x, dt) = truth(3.0);
    let mut worst = 0.0_f64;
    for i in 0..25 {
        let logger = Logger {
            rate: RATE,
            phase: f64::from(i) / 25.0 / RATE,
            noise: 0.0,
        };
        let s = sample(&x, dt, &logger, &mut Rng::new(1));
        let d = detect(&s, RATE, logger.phase);
        assert_eq!(d.events.len(), 1, "exactly one event");
        worst = worst.max((d.events[0].arrival.value() - 3.0).abs());
    }
    println!(
        "  largest timing error over all phases: {:.2e} s ({:.4} of a sample)",
        worst,
        worst * RATE
    );
    println!("  (limited by the 0.1 ms simulation step, not by the method)");
    check("error below 1% of a sample", worst * RATE < 0.01);
}

#[test]
fn with_noise_the_error_matches_the_reported_uncertainty() {
    // 2 cm of noise: typical of a 0-16 bar transducer. 400 trials with
    // different noise and clock phase.
    let (x, dt) = truth(3.0);
    let mut rng = Rng::new(2026);
    let (mut sum_sq, mut within_2sigma, mut sigma_sum) = (0.0, 0_u32, 0.0);
    let trials = 400;
    for _ in 0..trials {
        let logger = Logger {
            rate: RATE,
            phase: rng.uniform() / RATE,
            noise: 0.02,
        };
        let s = sample(&x, dt, &logger, &mut rng);
        let d = detect(&s, RATE, logger.phase);
        assert_eq!(d.events.len(), 1, "exactly one event per trial");
        let e = d.events[0];
        let err = e.arrival.value() - 3.0;
        sum_sq += err * err;
        sigma_sum += e.arrival_uncertainty.value();
        if err.abs() <= 2.0 * e.arrival_uncertainty.value() {
            within_2sigma += 1;
        }
    }
    let rms = (sum_sq / f64::from(trials)).sqrt();
    let reported = sigma_sum / f64::from(trials);
    let coverage = f64::from(within_2sigma) / f64::from(trials);
    println!("4.5 m drop, 2 cm noise, {trials} trials at random clock phases:");
    println!(
        "  RMS timing error    {:.3} ms  ({:.3} of a sample, {:.2} m of location)",
        rms * 1e3,
        rms * RATE,
        rms * 1000.0 / 2.0
    );
    println!("  reported 1-sigma    {:.3} ms (average)", reported * 1e3);
    println!(
        "  errors within 2 x reported sigma: {:.1}%",
        coverage * 100.0
    );
    check("timing error under a tenth of a sample", rms * RATE < 0.1);
    check(
        "the reported uncertainty is honest (>= 90% inside 2 sigma)",
        coverage >= 0.90,
    );
}
