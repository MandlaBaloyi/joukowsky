//! Steady state: the everyday flows and pressures (milestone S4).
//!
//! Code in `crates/domain/hydraulics/src/steady/`. Each test is a textbook
//! problem with an independent answer: a closed-form formula, or (for three
//! reservoirs) a bisection search written here from scratch.
//!
//! Friction in all of them: `h_f` = R Q|Q|, R = f L / (2 g D A^2).

mod check_valves;
mod closed_form;
mod three_reservoirs;

use jk_hydraulics::Pipe;
use jk_kernel::STANDARD_GRAVITY;

/// R = f L / (2 g D A^2) of a pipe, computed independently of the engine.
#[must_use]
pub fn resistance(p: &Pipe) -> f64 {
    let (d, a) = (p.diameter().value(), p.area().value());
    p.friction_factor() * p.length().value() / (2.0 * STANDARD_GRAVITY.value() * d * a * a)
}
