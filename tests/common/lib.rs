//! Shared helpers for the joukowsky validation tests.
//!
//! Every test prints what it measured next to what the physics predicts, so
//! a test log reads like a lab notebook, not just "ok".

pub mod estate;
pub mod refill;
pub mod sensor;

use jk_hydraulics::{Pipe, ReservoirPipeValve, ValveClosure};
use jk_kernel::{Head, Length, STANDARD_GRAVITY, Velocity, WaveSpeed};

/// The gravity the docs use in their worked numbers (they round to 9.81).
pub const G_DOCS: f64 = 9.81;

/// Asserts `actual` is within relative tolerance `rel` of `expected`, and
/// prints both so the log shows the comparison.
///
/// # Panics
/// If the values differ by more than the tolerance.
pub fn check_close(what: &str, actual: f64, expected: f64, rel: f64) {
    let scale = actual.abs().max(expected.abs()).max(f64::MIN_POSITIVE);
    let error = (actual - expected).abs() / scale;
    println!("  {what:<44} got {actual:>14.6}   expected {expected:>14.6}   rel.err {error:.1e}");
    assert!(
        error <= rel,
        "{what}: got {actual}, expected {expected} (relative error {error:.3e} > {rel:.1e})"
    );
}

/// Asserts `condition`, printing the claim being checked.
///
/// # Panics
/// If `condition` is false.
pub fn check(what: &str, condition: bool) {
    println!("  {what:<44} {}", if condition { "yes" } else { "NO" });
    assert!(condition, "{what}: not true");
}

/// A pipe from plain numbers: length [m], internal diameter [m], wave speed
/// [m/s], Darcy friction factor.
///
/// # Panics
/// If the numbers are not physical.
#[must_use]
pub fn pipe(length: f64, diameter: f64, wave_speed: f64, friction: f64) -> Pipe {
    Pipe::new(
        Length::new(length),
        Length::new(diameter),
        WaveSpeed::new(wave_speed),
        friction,
    )
    .expect("test pipe must be physical")
}

/// Steady flow velocity used by the standard single-pipe system [m/s].
pub const V0: f64 = 1.0;

/// The standard test system: a 100 m reservoir, 1000 m of DN300 pipe with
/// a = 1000 m/s, flowing at 1 m/s through a valve into a 50 m head.
/// Timing: 2L/a = 2 s, period 4L/a = 4 s.
///
/// # Panics
/// Never for these fixed, valid parameters.
#[must_use]
pub fn standard_system(friction: f64, closure: ValveClosure, reaches: u32) -> ReservoirPipeValve {
    let pipe = Pipe::new(
        Length::new(1000.0),
        Length::new(0.3),
        WaveSpeed::new(1000.0),
        friction,
    )
    .expect("standard pipe is valid");
    ReservoirPipeValve {
        reservoir_head: Head::new(100.0),
        initial_flow: Velocity::new(V0) * pipe.area(),
        pipe,
        downstream_head: Head::new(50.0),
        closure,
        reaches,
        gravity: STANDARD_GRAVITY,
    }
}
