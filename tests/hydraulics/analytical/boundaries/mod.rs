//! Boundaries: what a pressure wave does when it meets a fitting.
//!
//! Code in `crates/domain/hydraulics/src/boundary/`. The method: send a wave
//! of known amplitude F at the fitting and measure what is transmitted and
//! reflected, against `docs/02_physics_of_transients/joukowsky_equation.txt`
//! section 6.
//!
//! How to launch a wave by hand: in an undisturbed pipe (head H0, no flow),
//! a wave of height F travelling towards the fitting also moves the water
//! by dQ = F / B. The C+ value it carries to the fitting is therefore
//! `C_P` = (H0 + F) + B (F / B) = H0 + 2F.

mod burst;
mod check_valve;
mod reflections;

/// Undisturbed head in every boundary test [m].
pub const H0: f64 = 100.0;
/// Amplitude of the incoming test wave [m].
pub const F: f64 = 10.0;

/// The C+ value a wave of height `F` carries to a downstream fitting.
#[must_use]
pub fn incoming_c_plus() -> f64 {
    H0 + 2.0 * F
}
