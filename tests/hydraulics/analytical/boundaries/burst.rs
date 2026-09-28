//! A burst mid-pipe: the negative step it sends out is the engine's locator
//! beacon, and its size tells us how much water is being lost.
//!
//! `docs/02_physics_of_transients/joukowsky_equation.txt` section 5: seen from
//! the burst, the two pipe halves are in parallel (impedance B/2), so the
//! head drops by exactly (B/2) `Q_L`. The engine inverts this to size a
//! burst from a measured step - checked here against the leak boundary.

use jk_hydraulics::boundary::junction::{Arrival, PipeEnd};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Pipe, joukowsky};
use jk_kernel::{Area, Head, Length, STANDARD_GRAVITY, Velocity, WaveSpeed};
use jk_tests::{check, check_close};

/// DN150 asbestos-cement main, a = 1000 m/s (the docs' example pipe).
fn main_pipe() -> Pipe {
    Pipe::new(
        Length::new(500.0),
        Length::new(0.15),
        WaveSpeed::new(1000.0),
        0.0,
    )
    .expect("valid pipe")
}

/// The two halves of a pipe meeting at the burst node, flowing at q0
/// under head h0 just before the burst.
fn halves(h0: f64, q0: f64, b: f64) -> [Arrival; 2] {
    [
        Arrival {
            characteristic: h0 + b * q0,
            impedance: b,
            end: PipeEnd::Downstream,
        },
        Arrival {
            characteristic: h0 - b * q0,
            impedance: b,
            end: PipeEnd::Upstream,
        },
    ]
}

#[test]
fn a_10mm_hole_at_60m_head() {
    let pipe = main_pipe();
    let b = pipe.impedance(STANDARD_GRAVITY);
    let (h0, q0) = (60.0, (Velocity::new(0.5) * pipe.area()).value());
    let hole = Area::of_circle(Length::new(0.010));
    let k = leak::coefficient(0.6, hole, STANDARD_GRAVITY);

    println!("DN150 AC main at 60 m head, 0.5 m/s; a 10 mm hole opens (Cd 0.6):");
    let (h, ql) = leak::head(&halves(h0, q0, b), 0.0, k, 0.0);
    println!(
        "  water lost              {:.3} l/s  ({:.0} m^3 per day)",
        ql * 1e3,
        ql * 86_400.0
    );
    println!("  head step at the burst  {:.3} m", h - h0);

    check_close("step = -(B/2) Q_L [m]", h - h0, -b / 2.0 * ql, 1e-12);
    check_close(
        "orifice law at the new head [l/s]",
        ql * 1e3,
        k * h.sqrt() * 1e3,
        1e-12,
    );
    let sized = joukowsky::burst_flow_from_head_drop(
        Head::new(h - h0),
        pipe.wave_speed(),
        pipe.area(),
        STANDARD_GRAVITY,
    );
    check_close(
        "engine sizes the burst from the step [l/s]",
        sized.value() * 1e3,
        ql * 1e3,
        1e-12,
    );
}

#[test]
fn a_down_surge_turns_the_hole_into_an_intake() {
    let pipe = main_pipe();
    let b = pipe.impedance(STANDARD_GRAVITY);
    let k = leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY);
    let trench_water = 0.5;

    println!("Pump trip pulls the main to -3 m while trench water stands at +0.5 m:");
    let (h, ql) = leak::head(&halves(-3.0, 0.0, b), 0.0, k, trench_water);
    println!("  head at the hole        {h:.3} m");
    println!(
        "  flow through the hole   {:.4} l/s (negative = drawn IN)",
        ql * 1e3
    );
    check("water flows into the pipe (intrusion)", ql < 0.0);
    check_close(
        "orifice law, reversed [l/s]",
        -ql * 1e3,
        k * (trench_water - h).sqrt() * 1e3,
        1e-12,
    );
    println!("  => every such event is logged as a water-quality event (docs: P5 exposure).");
}
