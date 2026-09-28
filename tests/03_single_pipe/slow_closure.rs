//! V-A2 (coarse): close the valve slowly - 10 times the 2L/a round trip.
//!
//! The relief wave now returns from the reservoir while the valve is still
//! closing and cancels most of the rise. The surge must be far below
//! Joukowsky but not zero. Michaud's rough estimate is
//! dH ~ 2 L V0 / (g Tc) = Joukowsky x (2L/a) / Tc = Joukowsky / 10.
//! This is the physics behind principle P4: close slower than 2L/a.

use jk_hydraulics::{ValveClosure, joukowsky};
use jk_kernel::{Time, Velocity};
use jk_tests::{V0, check, standard_system};

#[test]
fn slow_closure_surge_is_well_below_joukowsky() {
    let s = standard_system(0.0, ValveClosure::linear(Time::new(20.0)), 20);
    let dh = joukowsky::head_change(Velocity::new(-V0), s.pipe.wave_speed(), s.gravity).value();
    let trace = s.simulate(Time::new(30.0), &[s.valve_node()]).unwrap();
    let rise = trace.max_head(0).value() - 100.0;

    println!("Linear closure over 20 s (10 x 2L/a):");
    println!("  Joukowsky (instant closure)  {dh:.3} m");
    println!("  Michaud estimate             {:.3} m", dh / 10.0);
    println!("  simulated surge              {rise:.3} m");
    check(
        "surge is positive (above 5% of Joukowsky)",
        rise > 0.05 * dh,
    );
    check("surge is below half of Joukowsky", rise < 0.5 * dh);
}
