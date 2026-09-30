//! The night watch on 90 simulated days of meter readings (`zone_nights` in
//! main.rs: daily pattern, nights varying by 0.3 l/s, 1% meter noise).

use jk_balance::{LeakOptions, smallest_detectable, watch};
use jk_tests::check;
use jk_tests::sensor::Rng;

use crate::zone_nights;

const DAYS: usize = 90;

#[test]
fn false_alarms_are_rare_on_a_zone_without_leaks() {
    let options = LeakOptions::default();
    let (mut alarms, mut nights) = (0usize, 0usize);
    for seed in 0..200 {
        let mut rng = Rng::new(seed);
        let n = zone_nights(DAYS, 0.0, 0, &mut rng);
        let w = watch(&n, &options).unwrap();
        alarms += w.alarms.len();
        nights += n.len() - options.baseline_nights;
    }
    #[allow(clippy::cast_precision_loss)] // counts are small
    let per = nights as f64 / alarms.max(1) as f64;
    println!(
        "200 zones x {DAYS} days, no leak: {alarms} false alarm(s) in {nights} watched nights \
         - one every {per:.0} nights"
    );
    check("fewer than one false alarm a year per zone", per > 365.0);
}

#[test]
fn a_one_litre_per_second_leak_is_caught_dated_and_sized() {
    let options = LeakOptions::default();
    let mut rng = Rng::new(7);
    let nights = zone_nights(DAYS, 0.001, 45, &mut rng);
    let w = watch(&nights, &options).unwrap();
    println!(
        "Night flow baseline {:.2} l/s, nights vary by {:.2} l/s; a week catches leaks over \
         {:.2} l/s",
        w.baseline.value() * 1000.0,
        w.spread.value() * 1000.0,
        w.smallest_in_a_week.value() * 1000.0
    );
    let a = *w.alarms.first().expect("the leak is caught");
    let night_of = |t: jk_kernel::Time| ((t.value() - crate::START) / 86_400.0).floor();
    println!(
        "  leak from mid-day 45: first night {} (the night of day {}), alarm night {}, size \
         {:.2} +/- {:.2} l/s",
        night_of(a.first_night),
        night_of(a.first_night),
        night_of(a.raised),
        a.size.value() * 1000.0,
        a.uncertainty.value() * 1000.0
    );
    check(
        "dated to the first night after it began (day 46)",
        (night_of(a.first_night) - 46.0).abs() < 0.5,
    );
    check("raised within 3 nights", night_of(a.raised) - 46.0 <= 3.0);
    check(
        "size within 2 uncertainties of 1 l/s",
        (a.size.value() - 0.001).abs() <= 2.0 * a.uncertainty.value(),
    );
    check("one alarm, no echoes", w.alarms.len() == 1);
}

#[test]
fn the_detection_curve_matches_the_cusum_arithmetic() {
    let options = LeakOptions::default();
    // The spread of this zone's nights, from leak-free runs.
    let mut rng = Rng::new(99);
    let spread = watch(&zone_nights(DAYS, 0.0, 0, &mut rng), &options)
        .unwrap()
        .spread;
    let d_min = smallest_detectable(spread, 7, &options).value();
    println!(
        "Night spread {:.3} l/s: the arithmetic says leaks over {:.3} l/s show within a week.",
        spread.value() * 1000.0,
        d_min * 1000.0
    );
    let mut rates = Vec::new();
    for factor in [0.5, 1.0, 1.5, 2.0] {
        let leak = factor * d_min;
        let mut caught = 0;
        for seed in 0..100 {
            let mut rng = Rng::new(1000 + seed);
            let n = zone_nights(DAYS, leak, 45, &mut rng);
            let w = watch(&n, &options).unwrap();
            let start = crate::START + 46.0 * 86_400.0;
            if w.alarms.iter().any(|a| {
                a.raised.value() >= start - 1.0 && a.raised.value() <= start + 6.5 * 86_400.0
            }) {
                caught += 1;
            }
        }
        println!(
            "  leak {:.2} l/s ({factor} x the arithmetic): caught within a week in {caught} of 100 zones",
            leak * 1000.0
        );
        rates.push(caught);
    }
    check(
        "more leak, more often caught",
        rates.windows(2).all(|w| w[1] >= w[0]),
    );
    check(
        "twice the arithmetic's size: caught 90%+ of the time",
        rates[3] >= 90,
    );
    check(
        "at the arithmetic's size: caught at least a third of the time",
        rates[1] >= 33,
    );
}
