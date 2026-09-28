//! Leak or burst at a node: an opening whose outflow depends on the head.
//!
//! `docs/02_physics_of_transients/governing_equations_and_moc.txt` section 2
//! and `column_separation_and_intrusion.txt` section 3. The opening obeys
//! the orifice law, in BOTH directions:
//!
//! ```text
//!   Q_L = +k sqrt(H - H_out)    when H > H_out   (water escapes)
//!   Q_L = -k sqrt(H_out - H)    when H < H_out   (INTRUSION: water outside
//!                                                  the pipe is drawn in)
//!   k   = Cd A_L sqrt(2 g)
//! ```
//!
//! `H_out` is the head outside the opening (ground level for a burst into
//! soil, the trench water level for intrusion). The two-way law matters:
//! during a down-surge, contaminated trench water enters through the same
//! hole - a water-quality event, not just a structural one.
//!
//! A burst is a leak whose opening appears suddenly: the solver sets k from
//! zero to its final value at the burst time. Pressure-dependent leak AREA
//! (FAVAD, van Zyl-Cassa modified orifice) is a later refinement
//! (`pressure_leakage_and_fatigue.txt` section 1).

use jk_kernel::{Acceleration, Area};

use crate::boundary::junction::{Arrival, sums};

/// Orifice coefficient k = Cd A sqrt(2 g) [m^2.5/s] of an opening with
/// discharge coefficient `cd` (about 0.6 for a sharp-edged hole) and area
/// `area`.
#[must_use]
pub fn coefficient(discharge_coefficient: f64, area: Area, gravity: Acceleration) -> f64 {
    discharge_coefficient * area.value() * (2.0 * gravity.value()).sqrt()
}

/// Head at a junction node that also has a leak (coefficient `k`, outside
/// head `outside_head`) and a fixed `demand`. Returns (head, leak flow);
/// the leak flow is negative during intrusion.
///
/// Solves `S H + Q_L(H) = C` exactly (S = sum 1/B, C = sum C/B - demand).
/// With u = sqrt(|H - `H_out`|) it is the quadratic S u^2 + k u = |d|,
/// d = C - S `H_out`, solved in the form without cancellation:
/// u = 2|d| / (k + sqrt(k^2 + 4 S |d|)).
///
/// # Panics
/// In debug builds, if `arrivals` is empty or `k` is negative.
#[must_use]
pub fn head(arrivals: &[Arrival], demand: f64, k: f64, outside_head: f64) -> (f64, f64) {
    debug_assert!(!arrivals.is_empty(), "a leak node needs at least one pipe");
    debug_assert!(k >= 0.0, "leak coefficient must be >= 0");
    let (c, s) = sums(arrivals);
    let d = (c - demand) - s * outside_head;
    if d == 0.0 {
        return (outside_head, 0.0);
    }
    let u = 2.0 * d.abs() / (k + (k * k + 4.0 * s * d.abs()).sqrt());
    let sign = d.signum();
    (outside_head + sign * u * u, sign * k * u)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boundary::junction::PipeEnd;

    /// A leak mid-pipe: the pipe's two halves meet at the leak node.
    fn mid_pipe(h0: f64, q0: f64, b: f64) -> [Arrival; 2] {
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
    fn outflow_obeys_the_orifice_law_and_continuity() {
        let arrivals = mid_pipe(80.0, 0.05, 5000.0);
        let (k, h_out) = (0.004, 0.0);
        let (h, ql) = head(&arrivals, 0.0, k, h_out);
        assert!(ql > 0.0);
        assert!((ql - k * (h - h_out).sqrt()).abs() < 1e-15, "orifice law");
        let net = arrivals[0].flow(h) - arrivals[1].flow(h);
        assert!((net - ql).abs() < 1e-15, "continuity");
    }

    #[test]
    fn below_the_outside_head_water_is_drawn_in() {
        let arrivals = mid_pipe(-5.0, 0.0, 5000.0);
        let (k, h_out) = (0.004, 1.0);
        let (h, ql) = head(&arrivals, 0.0, k, h_out);
        assert!(ql < 0.0, "expected intrusion, got {ql}");
        assert!((ql + k * (h_out - h).sqrt()).abs() < 1e-15);
    }

    #[test]
    fn a_zero_coefficient_is_no_leak() {
        let arrivals = mid_pipe(80.0, 0.05, 5000.0);
        let (h, ql) = head(&arrivals, 0.0, 0.0, 0.0);
        assert!(ql.abs() < f64::EPSILON);
        assert!((h - 80.0).abs() < 1e-12);
    }
}
