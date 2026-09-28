//! The compatibility equations along the characteristic lines dx/dt = +a
//! (C+) and dx/dt = -a (C-).
//!
//! `docs/02_physics_of_transients/governing_equations_and_moc.txt`
//! section 2. With B = a / (g A) and R = f dx / (2 g D A^2), for node P at
//! the new time between neighbours A (left) and B (right) at the old time:
//!
//! ```text
//!   C+ : H_P = C_P - B Q_P,   C_P = H_A + B Q_A - R Q_A |Q_A|
//!   C- : H_P = C_M + B Q_P,   C_M = H_B - B Q_B + R Q_B |Q_B|
//! ```
//!
//! Every MOC solver in the crate - single pipe, network, and the boundary
//! devices - uses these same functions, so they are tested once and trusted
//! everywhere.

use jk_kernel::{Acceleration, Length};

use crate::Pipe;
use crate::pipe::friction::quasi_steady;

/// The coefficients B and R of one pipe on its grid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Characteristics {
    impedance: f64,
    friction: f64,
}

impl Characteristics {
    /// Coefficients for `pipe` cut into reaches of length `reach_length`.
    #[must_use]
    pub fn new(pipe: &Pipe, reach_length: Length, gravity: Acceleration) -> Self {
        Self {
            impedance: pipe.impedance(gravity),
            friction: quasi_steady::coefficient(
                pipe.friction_factor(),
                reach_length,
                pipe.diameter(),
                gravity,
            ),
        }
    }

    /// Characteristic impedance B = a / (g A) [s/m^2].
    #[must_use]
    pub fn impedance(&self) -> f64 {
        self.impedance
    }

    /// Friction coefficient per reach R [s^2/m^5].
    #[must_use]
    pub fn friction(&self) -> f64 {
        self.friction
    }

    /// `C_P` carried along C+ from the node to the LEFT (head, flow).
    #[must_use]
    #[inline]
    pub fn c_plus(&self, head: f64, flow: f64) -> f64 {
        head + self.impedance * flow - self.friction * flow * flow.abs()
    }

    /// `C_M` carried along C- from the node to the RIGHT (head, flow).
    #[must_use]
    #[inline]
    pub fn c_minus(&self, head: f64, flow: f64) -> f64 {
        head - self.impedance * flow + self.friction * flow * flow.abs()
    }

    /// Interior node: both characteristics arrive, giving (head, flow).
    #[must_use]
    #[inline]
    pub fn interior(&self, c_plus: f64, c_minus: f64) -> (f64, f64) {
        (
            f64::midpoint(c_plus, c_minus),
            (c_plus - c_minus) / (2.0 * self.impedance),
        )
    }

    /// Steady friction head loss over one reach, R Q|Q|.
    #[must_use]
    #[inline]
    pub fn reach_head_loss(&self, flow: f64) -> f64 {
        self.friction * flow * flow.abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::{Length, STANDARD_GRAVITY, WaveSpeed};

    fn chars(friction_factor: f64) -> Characteristics {
        let pipe = Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(1000.0),
            friction_factor,
        )
        .unwrap();
        Characteristics::new(&pipe, Length::new(50.0), STANDARD_GRAVITY)
    }

    #[test]
    fn interior_solution_lies_on_both_characteristics() {
        let c = chars(0.02);
        let (cp, cm) = (c.c_plus(101.0, 0.07), c.c_minus(99.0, 0.06));
        let (h, q) = c.interior(cp, cm);
        let b = c.impedance();
        assert!((h - (cp - b * q)).abs() < 1e-12, "C+ violated");
        assert!((h - (cm + b * q)).abs() < 1e-12, "C- violated");
    }

    #[test]
    fn a_steady_friction_gradient_is_a_fixed_point() {
        // Uniform flow with head falling by R Q|Q| per reach must reproduce
        // itself exactly at the middle node.
        let c = chars(0.02);
        let q = 0.07;
        let dh = c.reach_head_loss(q);
        let (h_left, h_right) = (100.0, 100.0 - 2.0 * dh);
        let (h, q_new) = c.interior(c.c_plus(h_left, q), c.c_minus(h_right, q));
        assert!((h - (100.0 - dh)).abs() < 1e-12);
        assert!((q_new - q).abs() < 1e-15);
    }
}
