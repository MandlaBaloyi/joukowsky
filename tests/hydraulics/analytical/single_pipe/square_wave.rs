//! V-A1: shut the valve instantly in a frictionless pipe.
//!
//! Theory: the head at the valve jumps by exactly dH = a V0 / g (Joukowsky),
//! stays there while the wave travels to the reservoir and back (2L/a),
//! drops to the same amount BELOW the reservoir head for another 2L/a, and
//! repeats forever with period 4L/a - a perfect square wave, because
//! nothing removes energy.

use jk_hydraulics::{ValveClosure, joukowsky};
use jk_kernel::{Time, Velocity};
use jk_tests::{V0, check, check_close, standard_system};

#[test]
fn va1_frictionless_instant_closure_is_a_joukowsky_square_wave() {
    let s = standard_system(0.0, ValveClosure::Instantaneous, 20);
    let dh = joukowsky::head_change(Velocity::new(-V0), s.pipe.wave_speed(), s.gravity).value();
    let trace = s.simulate(Time::new(12.0), &[s.valve_node()]).unwrap();
    let heads = trace.head(0);
    let hr = 100.0;

    println!("Grid and timing:");
    check_close("time step dt [s]", trace.time_step().value(), 0.05, 1e-15);
    println!("  2L/a = 2 s = 40 steps; period 4L/a = 4 s = 80 steps.");
    println!("  The valve shuts at step 1 (t = dt), so each front lands one step");
    println!("  after its ideal time: high for steps 1..=40, low for 41..=80.");

    println!("Levels at the valve:");
    check_close(
        "head just after closure [m]",
        heads[1].value(),
        hr + dh,
        1e-12,
    );
    check_close(
        "head after the wave returns [m]",
        heads[41].value(),
        hr - dh,
        1e-12,
    );
    check_close(
        "head one period later [m]",
        heads[81].value(),
        hr + dh,
        1e-12,
    );

    println!(
        "Every one of the {} samples on the square wave:",
        heads.len()
    );
    let mut worst = 0.0_f64;
    for (k, h) in heads.iter().enumerate().skip(1) {
        let expected = if (k - 1) % 80 < 40 { hr + dh } else { hr - dh };
        worst = worst.max((h.value() - expected).abs());
    }
    println!("  largest deviation from the ideal square wave: {worst:.2e} m");
    check("all samples within 1e-9 m of the ideal", worst < 1e-9);

    let leak = trace
        .flow(0)
        .iter()
        .skip(1)
        .map(|q| q.value().abs())
        .fold(0.0, f64::max);
    check("no flow through the shut valve", leak < 1e-15);
}
