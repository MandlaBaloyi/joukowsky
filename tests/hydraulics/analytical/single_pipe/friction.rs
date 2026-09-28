//! Friction changes the story in two ways the engine must reproduce:
//!
//! 1. Line packing: after the first front, the head at the valve keeps
//!    CLIMBING, because friction had been holding the upstream water at a
//!    higher head. This is why Joukowsky is not an absolute upper bound in
//!    long pipes (`joukowsky_equation.txt` section 3).
//! 2. Damping: each oscillation is smaller than the last.

use jk_hydraulics::{ValveClosure, joukowsky};
use jk_kernel::{Head, Time, Velocity};
use jk_tests::{V0, check, standard_system};

#[test]
fn friction_causes_line_packing_and_damping() {
    let s = standard_system(0.03, ValveClosure::Instantaneous, 20);
    let dh = joukowsky::head_change(Velocity::new(-V0), s.pipe.wave_speed(), s.gravity).value();
    let trace = s.simulate(Time::new(16.0), &[s.valve_node()]).unwrap();
    let h = trace.head(0);
    let steady = s.steady_valve_head().value();

    println!("Friction f = 0.03, instant closure; Joukowsky dH = {dh:.3} m");
    println!("  steady valve head            {steady:.3} m");
    println!("  just after closure (step 1)  {:.3} m", h[1].value());
    println!("  just before relief (step 40) {:.3} m", h[40].value());
    check("head climbs during 2L/a (line packing)", h[40] > h[1]);
    check(
        "surge exceeds Joukowsky",
        trace.max_head(0).value() - steady > dh,
    );

    let peak = |from: usize, to: usize| {
        h[from..to]
            .iter()
            .copied()
            .fold(Head::new(f64::MIN), Head::max)
    };
    let (first, third) = (peak(0, 80), peak(160, 240));
    println!("  peak in period 1             {:.3} m", first.value());
    println!("  peak in period 3             {:.3} m", third.value());
    check("oscillation decays", third < first);
}
