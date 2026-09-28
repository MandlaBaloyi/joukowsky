//! Rotating inertia from Thorley's regressions, when the datasheet is silent.

use jk_hydraulics::thorley_inertia;
use jk_tests::{check, check_close};

use super::rated_power_kw;

#[test]
fn inertia_of_the_station_pump() {
    let (p, n) = (rated_power_kw(), 1450.0_f64);
    let total = thorley_inertia(p, n);
    println!("Pump + motor for {p:.1} kW at {n} rpm:");
    println!("  estimated inertia {total:.2} kg m^2");

    // The same regressions as usually printed, with N in thousands of rpm:
    // I_pump = 0.03768 (P/N^3)^0.9556, I_motor = 0.0043 (P/N)^1.48.
    let k = n / 1000.0;
    let pump = 0.03768 * (p / k.powi(3)).powf(0.9556);
    let motor = 0.0043 * (p / k).powf(1.48);
    check_close(
        "agrees with the 1000-rpm form [kg m^2]",
        total,
        pump + motor,
        0.01,
    );
    check(
        "in the plausible range for ~80 kW (1-5 kg m^2)",
        (1.0..5.0).contains(&total),
    );
    println!("  NOTE: real values scatter -50% .. +100% around this estimate;");
    println!("  the engine treats it as a prior to learn from trip events, not a fact.");
}
