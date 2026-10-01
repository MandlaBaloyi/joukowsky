//! V-A19: rainflow counting and fatigue damage.

use jk_health::{Counter, Cycle, SnCurve, equivalent_cycles, miner, rainflow};
use jk_tests::sensor::Rng;
use jk_tests::{check, check_close};

/// Cycles counted at `range` (to 1e-9).
fn at(cycles: &[Cycle], range: f64) -> f64 {
    cycles
        .iter()
        .filter(|c| (c.range - range).abs() < 1e-9)
        .map(|c| c.count)
        .sum()
}

#[test]
fn va19_the_standards_worked_example() {
    // ASTM E1049-85, the rainflow example: load history -2, 1, -3, 5, -1,
    // 3, -4, 4, -2 [VERIFY the figure number against the standard].
    let history = [-2.0, 1.0, -3.0, 5.0, -1.0, 3.0, -4.0, 4.0, -2.0];
    let cycles = rainflow(&history, 0.0).unwrap();
    println!("ASTM E1049 example: -2, 1, -3, 5, -1, 3, -4, 4, -2");
    println!("  range   counted   the standard");
    for (range, expected) in [(3.0, 0.5), (4.0, 1.5), (6.0, 0.5), (8.0, 1.0), (9.0, 0.5)] {
        check_close(
            &format!("  {range:5.0}"),
            at(&cycles, range),
            expected,
            1e-12,
        );
    }
    let total: f64 = cycles.iter().map(|c| c.count).sum();
    check_close("  total cycles", total, 4.0, 1e-12);
}

#[test]
fn va19_a_steady_sine_is_n_cycles_of_twice_its_amplitude() {
    // n periods of a sine of amplitude 10 m, starting and ending at 0. The
    // three-point method pairs each new extreme with the last: 2n - 2 half
    // cycles of 20 m, and at the end one more half of 20 m and the two
    // halves of 10 m at the ends.
    let (n, per) = (50, 64);
    let x: Vec<f64> = (0..=n * per)
        .map(|k| 10.0 * (std::f64::consts::TAU * f64::from(k) / f64::from(per)).sin())
        .collect();
    let cycles = rainflow(&x, 0.0).unwrap();
    println!("{n} periods of a 10 m sine:");
    check_close(
        "  cycles of 20 m: n - 1/2",
        at(&cycles, 20.0),
        f64::from(n) - 0.5,
        1e-12,
    );
    check_close(
        "  and of 10 m: the two half-swings at the ends",
        at(&cycles, 10.0),
        1.0,
        1e-12,
    );
    // Miner: S-N through 1e6 cycles at 20 m, m = 4.
    let curve = SnCurve {
        reference_range: 20.0,
        reference_cycles: 1e6,
        exponent: 4.0,
    };
    let expected = (f64::from(n) - 0.5) / 1e6 + 1.0 / (1e6 * 16.0);
    check_close(
        "  Miner's damage = sum n / N(S), N(10 m) = 16 N(20 m) [-]",
        miner(&cycles, &curve),
        expected,
        1e-12,
    );
}

#[test]
fn va19_equivalent_cycles_rank_by_the_exponent() {
    // Two blocks: 1000 swings of 10 m and 10 of 50 m. With m = 4 the ten
    // big ones outweigh the thousand small: 10 x 5^4 = 6250 against 1000.
    let mut x = Vec::new();
    for _ in 0..1000 {
        x.extend([50.0, 60.0]);
    }
    for _ in 0..10 {
        x.extend([50.0, 100.0]);
    }
    x.push(50.0);
    let cycles = rainflow(&x, 0.0).unwrap();
    let n_eq = equivalent_cycles(&cycles, 10.0, 4.0);
    println!("1000 swings of 10 m and 10 of 50 m, as swings of 10 m (m = 4):");
    check_close(
        "  equivalent cycles = 1000 + 10 x 5^4",
        n_eq,
        1000.0 + 10.0 * 625.0,
        1e-12,
    );
}

#[test]
fn va19_live_in_pieces_counts_the_same_as_the_whole_record() {
    let mut rng = Rng::new(5);
    let x: Vec<f64> = (0..20_000)
        .map(|k| 50.0 + 20.0 * (f64::from(k) / 37.0).sin() + 3.0 * rng.normal())
        .collect();
    let whole = rainflow(&x, 0.5).unwrap();
    let mut live = Counter::new(0.5).unwrap();
    for piece in x.chunks(128) {
        live.push(piece).unwrap();
    }
    let live = live.finish();
    check(
        &format!(
            "  {} cycles, identical whole and live (128-sample pieces)",
            whole.len()
        ),
        whole == live,
    );
}

#[test]
fn va19_the_gate_keeps_the_swings_and_drops_the_noise() {
    // A 10 m sine as a logger records it: 2 cm noise on every sample.
    let mut rng = Rng::new(9);
    let (n, per) = (50, 128);
    let clean: Vec<f64> = (0..=n * per)
        .map(|k| 50.0 + 10.0 * (std::f64::consts::TAU * f64::from(k) / f64::from(per)).sin())
        .collect();
    let noisy: Vec<f64> = clean.iter().map(|x| x + 0.02 * rng.normal()).collect();
    let raw = rainflow(&noisy, 0.0).unwrap();
    let gated = rainflow(&noisy, 0.2).unwrap();
    println!("A 10 m sine with 2 cm of logger noise:");
    println!("  without a gate: {} cycles counted", raw.len());
    let big: f64 = gated
        .iter()
        .filter(|c| c.range > 19.0)
        .map(|c| c.count)
        .sum();
    check(
        &format!(
            "  gated at 0.2 m: {} cycles, all within noise of 20 m or 10 m",
            gated.len()
        ),
        gated
            .iter()
            .all(|c| (c.range - 20.0).abs() < 0.2 || (c.range - 10.0).abs() < 0.2),
    );
    check_close("  the 20 m cycles: n - 1/2", big, f64::from(n) - 0.5, 1e-12);
}
