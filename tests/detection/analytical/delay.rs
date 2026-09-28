//! How long does it take to raise the alarm?
//!
//! For a step of d noise units, CUSUM gains (d - k) evidence per sample and
//! alarms when it EXCEEDS h, so the alarm comes on sample
//! floor(h / (d - k)) + 1 of the step.

use jk_tests::check;

const RATE: f64 = 128.0;

#[test]
fn delay_follows_the_cusum_formula() {
    let (k, h) = (3.0_f64, 10.0_f64); // the defaults
    let sigma = 0.02;
    println!("Sharp drops at t = 3 s, noise-free signal with a 2 cm noise floor set:");
    for d in [5.0_f64, 8.0, 20.0, 200.0] {
        let step = d * sigma;
        // Noise-free: the detector's measured noise hits its floor, so set the
        // floor to sigma to control the units exactly.
        let config = jk_detection::DetectorConfig {
            min_noise: jk_kernel::Head::new(sigma),
            ..jk_detection::DetectorConfig::default()
        };
        let x: Vec<f64> = (0..768)
            .map(|i| if i < 384 { 60.0 } else { 60.0 - step })
            .collect();
        let det = crate::detect_with(&x, RATE, 0.0, config);
        let e = det.events[0];
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let expected = (h / (d - k)).floor() as usize + 1;
        let delay_samples = ((e.alarm.value() - 383.0 / RATE) * RATE).round();
        #[allow(clippy::cast_precision_loss)]
        let expected_f = expected as f64;
        println!(
            "  {d:>5} noise units ({step:.2} m): alarm after {delay_samples} samples (formula {expected})"
        );
        check(
            "  delay matches floor(h / (d - k)) + 1",
            (delay_samples - expected_f).abs() < 0.5,
        );
    }
}
