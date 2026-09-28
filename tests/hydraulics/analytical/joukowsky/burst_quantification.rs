//! Section 5: a burst is an "inverse valve". Its pressure step tells us how
//! much water it is losing - this is how the engine sizes a burst (P4).

use jk_hydraulics::joukowsky;
use jk_kernel::{Acceleration, Area, Head, Length, WaveSpeed};
use jk_tests::{G_DOCS, check_close};

/// DN150 asbestos-cement main, a = 1000 m/s: B ~ 5770 s/m^2, and a
/// 2 l/s burst gives a ~5.8 m (0.57 bar) negative step.
#[test]
fn dn150_ac_main_2_litre_per_second_burst() {
    println!("DN150 AC main, a = 1000 m/s:");
    let area = Area::of_circle(Length::new(0.15));
    let a = WaveSpeed::new(1000.0);
    let g = Acceleration::new(G_DOCS);

    let b = joukowsky::impedance(a, area, g);
    check_close("impedance B [s/m^2]", b, 5770.0, 1e-3);

    // Seen from the burst, two pipe halves in parallel: impedance B / 2.
    let step = Head::new(b / 2.0 * 0.002);
    check_close("step from a 2 l/s burst [m]", step.value(), 5.8, 0.01);

    let q = joukowsky::burst_flow_from_head_drop(-step, a, area, g);
    check_close(
        "burst flow back from the step [l/s]",
        q.value() * 1000.0,
        2.0,
        1e-12,
    );
    check_close(
        "water lost per day [m^3]",
        q.value() * 86_400.0,
        172.8,
        1e-12,
    );
}
