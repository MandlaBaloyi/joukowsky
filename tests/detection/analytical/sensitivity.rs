//! How big must a step be to be caught - and timed well enough to locate?
//!
//! The placement advisor asks, for every point on the network, whether a
//! burst there would be FELT at each logger. That needs the smallest step
//! the detector reliably finds. Measured here, not assumed (principle P8):
//! steps of several sizes, hidden in Gaussian noise, at a random moment
//! within a sample; "caught" means an event whose arrival is within 2
//! samples (16 ms at 128 S/s, about 8 m of pipe at a = 1000 m/s) of the
//! truth - good enough to locate.

use jk_tests::check;
use jk_tests::sensor::Rng;

use crate::detect;

const RATE: f64 = 128.0;
const NOISE: f64 = 0.02;

/// Share of `trials` in which a drop of `units` noise standard deviations
/// at t = 3 s (+ a random fraction of a sample) is caught and timed.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]
fn caught(units: f64, trials: u32, rng: &mut Rng) -> f64 {
    let mut hits = 0;
    for _ in 0..trials {
        let t_step = 3.0 + rng.uniform() / RATE;
        let x: Vec<f64> = (0..(5.0 * RATE) as usize)
            .map(|k| {
                // Integrating logger: sample k averages ((k-1)/R, k/R].
                let (a, b) = ((k as f64 - 1.0) / RATE, k as f64 / RATE);
                let covered = ((b - t_step) / (b - a)).clamp(0.0, 1.0);
                60.0 - units * NOISE * covered + NOISE * rng.normal()
            })
            .collect();
        let d = detect(&x, RATE, 0.0);
        if d.events
            .iter()
            .any(|e| (e.arrival.value() - t_step).abs() <= 2.0 / RATE)
        {
            hits += 1;
        }
    }
    f64::from(hits) / f64::from(trials)
}

#[test]
fn the_smallest_step_the_detector_catches_and_times() {
    let mut rng = Rng::new(42);
    println!("Step size (noise sd) -> share caught and timed to 2 samples (300 trials):");
    let mut table = Vec::new();
    for units in [3.0, 4.0, 5.0, 6.0, 8.0, 10.0] {
        let p = caught(units, 300, &mut rng);
        println!(
            "  {units:>4.0} sd ({:>4.0} mm)   {:>5.1}%",
            units * NOISE * 1000.0,
            p * 100.0
        );
        table.push((units, p));
    }
    let at = |u: f64| {
        table
            .iter()
            .find(|(v, _)| (v - u).abs() < 1e-9)
            .map_or(0.0, |(_, p)| *p)
    };
    check(
        "larger steps are caught at least as often",
        table.windows(2).all(|w| w[1].1 >= w[0].1 - 0.02),
    );
    // The placement advisor's default threshold (PlacementOptions::
    // detect_units = 6) rests on these two numbers.
    check("5 sd: caught at least 95% of the time", at(5.0) >= 0.95);
    check(
        "6 sd and more: always caught",
        at(6.0) >= 0.99 && at(8.0) >= 0.99,
    );
}
