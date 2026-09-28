//! Demand changes slowly through the day, moving pressure by metres over
//! minutes. That is not an event: the baseline must follow it. A burst on
//! top of the drift must still be caught.

use jk_tests::check;
use jk_tests::sensor::Rng;

use crate::detect;

const RATE: f64 = 128.0;

/// 10 minutes: 60 m +/- 3 m swinging with a 5-minute period, plus 2 cm noise;
/// optionally a 4.5 m burst drop at t = 400 s.
fn drifting(burst: bool) -> Vec<f64> {
    let mut rng = Rng::new(99);
    (0..76_800)
        .map(|i| {
            let t = f64::from(i) / RATE;
            let drift = 3.0 * (std::f64::consts::TAU * t / 300.0).sin();
            let step = if burst && t >= 400.0 { -4.5 } else { 0.0 };
            60.0 + drift + step + 0.02 * rng.normal()
        })
        .collect()
}

#[test]
fn slow_demand_drift_is_not_an_event() {
    let d = detect(&drifting(false), RATE, 0.0);
    println!(
        "10 minutes swinging +/- 3 m (5-minute period) with 2 cm noise: {} alarms",
        d.events.len()
    );
    check("drift raises no alarm", d.events.is_empty());
}

#[test]
fn a_burst_on_top_of_drift_is_still_caught() {
    let d = detect(&drifting(true), RATE, 0.0);
    println!("Same drift, with a 4.5 m drop at t = 400 s:");
    for e in &d.events {
        println!(
            "  event at {:.4} s, step {:.2} m",
            e.arrival.value(),
            e.step.value()
        );
    }
    check("exactly one event", d.events.len() == 1);
    let e = d.events[0];
    check(
        "timed within one sample of 400 s",
        (e.arrival.value() - 400.0).abs() < 1.0 / RATE,
    );
    check(
        "step measured within 5 cm",
        (e.step.value() + 4.5).abs() < 0.05,
    );
}
