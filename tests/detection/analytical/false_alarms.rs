//! "A system that raises ten false bursts a week will be switched off"
//! (`event_detection.txt` section 3). On pure sensor noise the detector must
//! stay silent.

use jk_tests::check;
use jk_tests::sensor::Rng;

use crate::detect;

const RATE: f64 = 128.0;

fn silent_on_noise(hours: f64, seed: u64) -> usize {
    let mut rng = Rng::new(seed);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (hours * 3600.0 * RATE) as usize;
    let x: Vec<f64> = (0..n).map(|_| 60.0 + 0.02 * rng.normal()).collect();
    detect(&x, RATE, 0.0).events.len()
}

#[test]
fn ten_minutes_of_pure_noise_raise_no_alarm() {
    let alarms = silent_on_noise(10.0 / 60.0, 11);
    println!("10 minutes of 2 cm Gaussian noise at 128 S/s (76 800 samples): {alarms} alarms");
    check("no false alarms", alarms == 0);
}

#[test]
#[ignore = "full suite: 24 hours of samples (11 million)"]
fn a_full_day_of_pure_noise_raises_no_alarm() {
    let alarms = silent_on_noise(24.0, 12);
    println!("24 hours of 2 cm Gaussian noise at 128 S/s (11 059 200 samples): {alarms} alarms");
    check("no false alarms in a day", alarms == 0);
    println!("  NOTE: real noise has heavier tails than Gaussian (taps, pumps, knocks);");
    println!("  false-alarm rates must be re-measured on field data (rung 4).");
}
