//! Quasi-steady Darcy-Weisbach friction: the loss the steady flow of the
//! same instantaneous rate would have.
//!
//! `docs/02_physics_of_transients/friction_and_damping.txt` section 1.
//! Known to under-predict the damping of transients; it is the reference
//! model the unsteady models are measured against.

use jk_kernel::{Acceleration, Area, Flow, Head, Length};

/// Head loss over a length of pipe: `h_f` = f L Q|Q| / (2 g D A^2).
///
/// Signed with the flow, so reversing the flow reverses the loss.
#[must_use]
pub fn head_loss(
    friction_factor: f64,
    length: Length,
    diameter: Length,
    flow: Flow,
    gravity: Acceleration,
) -> Head {
    let q = flow.value();
    Head::new(coefficient(friction_factor, length, diameter, gravity) * q * q.abs())
}

/// The coefficient R in `h_f` = R Q|Q|: R = f L / (2 g D A^2) [s^2/m^5].
///
/// With L = one MOC reach this is the R of the characteristic equations.
#[must_use]
pub fn coefficient(
    friction_factor: f64,
    length: Length,
    diameter: Length,
    gravity: Acceleration,
) -> f64 {
    let a = Area::of_circle(diameter).value();
    friction_factor * length.value() / (2.0 * gravity.value() * diameter.value() * a * a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::{STANDARD_GRAVITY, Velocity};

    #[test]
    fn matches_darcy_weisbach_in_velocity_form_and_is_odd_in_flow() {
        let (l, d) = (Length::new(1000.0), Length::new(0.3));
        let q = Velocity::new(1.0) * Area::of_circle(d);
        let hf = head_loss(0.02, l, d, q, STANDARD_GRAVITY);
        // f L V^2 / (2 g D) with V = 1 m/s.
        let expected = 0.02 * 1000.0 / (2.0 * STANDARD_GRAVITY.value() * 0.3);
        assert!((hf.value() - expected).abs() < 1e-12);
        assert_eq!(head_loss(0.02, l, d, -q, STANDARD_GRAVITY), -hf);
    }
}
