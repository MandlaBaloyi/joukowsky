//! Balance tests: meter readings in, slow-leak alarms out.
//!
//! Code under test: `crates/domain/balance` (zone balance, night flows, the
//! leak watch) - `docs/04_detection_and_localisation/slow_leaks_and_water_
//! balance.txt`.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only balance`

mod datasets;
mod simulation;

use jk_balance::{Direction, Meter, Night, NightWindow, balance, night_flows};
use jk_kernel::Time;
use jk_tests::sensor::Rng;

/// 15-minute readings.
pub const STEP: f64 = 900.0;
/// Readings a day.
pub const PER_DAY: usize = 96;
/// Local midnight (SAST) on 2026-09-01, in UTC seconds.
pub const START: f64 = 1_788_220_800.0 - 7200.0;

/// A synthetic zone: an inflow meter and an outflow meter (water passed
/// on to the next zone), `days` of readings. Consumption follows a daily
/// pattern with a night low of about 2 l/s that varies night to night
/// (0.3 l/s spread); each meter reading carries 1% random noise. A leak of
/// `leak` [m^3/s] runs from mid-day of day `leak_from` on.
///
/// # Panics
/// Never: the synthetic series are valid by construction.
#[must_use]
pub fn zone_nights(days: usize, leak: f64, leak_from: usize, rng: &mut Rng) -> Vec<Night> {
    let n = days * PER_DAY;
    let (mut inflow, mut outflow) = (Vec::with_capacity(n), Vec::with_capacity(n));
    let mut night_use = 0.002;
    for k in 0..n {
        let day = k / PER_DAY;
        #[allow(clippy::cast_precision_loss)] // under 96
        let hour = (k % PER_DAY) as f64 / 4.0;
        if k % PER_DAY == 0 {
            night_use = 0.002 + 0.0003 * rng.normal();
        }
        // Day-time use rises to ~25 l/s at 07:00 and 19:00.
        let peaks = (-((hour - 7.0) / 2.0).powi(2)).exp() + (-((hour - 19.0) / 2.5).powi(2)).exp();
        let used = night_use + 0.023 * peaks;
        let leaking = if day > leak_from || (day == leak_from && hour >= 12.0) {
            leak
        } else {
            0.0
        };
        let passed_on = 0.010;
        let into = used + leaking + passed_on;
        inflow.push(into * (1.0 + 0.01 * rng.normal()));
        outflow.push(passed_on * (1.0 + 0.01 * rng.normal()));
    }
    let meters = [
        Meter {
            name: "bulk in".into(),
            direction: Direction::In,
            flows: inflow,
        },
        Meter {
            name: "to next zone".into(),
            direction: Direction::Out,
            flows: outflow,
        },
    ];
    let b = balance(Time::new(START), Time::new(STEP), &meters, None).unwrap();
    night_flows(&b, NightWindow::default()).unwrap()
}
