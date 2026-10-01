//! Vapour cavities on the network grid: the discrete vapour cavity model
//! (DVCM; Wylie & Streeter 1993, review Bergant, Simpson & Tijsseling
//! 2006; `docs/02_physics_of_transients/column_separation_and_intrusion.txt`
//! section 5).
//!
//! Where a point's head would fall below its vapour floor (the floor as a
//! gauge head plus the pipe's elevation there), the water boils: the head
//! is held AT the floor, the two characteristics then give two different
//! flows - into the point from upstream (Qu, from C+) and out of it
//! downstream (Qd, from C-) - and the difference fills a vapour cavity:
//!
//! ```text
//!   H  = floor
//!   Qu = (C_P - H) / B,   Qd = (H - C_M) / B
//!   V  = V_old + dt (Qd - Qu)          (psi = 1: the new flows)
//! ```
//!
//! While V > 0 the cavity stays; when the water returns and V would fall
//! to 0 or below, it collapses: the point is solved as usual again, and the
//! columns meeting there make the collapse surge. At a node the same
//! balance uses every pipe's characteristic and the node's own outflow
//! (demand, leak, valve) at the floor head.
//!
//! The cavities are an exact water budget: water missing from the columns
//! is the cavity volume. psi = 1 [VERIFY against psi = 0.5 on Bergant &
//! Simpson's laboratory runs when that data is approved].

use crate::boundary::junction::{Arrival, PipeEnd};

/// One grid point inside a pipe, from the characteristics arriving at it
/// (`c_plus` = `C_P`, `c_minus` = `C_M`, impedance `b`), its piezometric
/// `floor` and its cavity `volume` before the step. Returns (head, flow on
/// the upstream side, flow on the downstream side, cavity volume).
#[must_use]
#[inline]
pub fn point(
    c_plus: f64,
    c_minus: f64,
    b: f64,
    floor: f64,
    volume: f64,
    dt: f64,
) -> (f64, f64, f64, f64) {
    let head = f64::midpoint(c_plus, c_minus);
    if volume > 0.0 || head < floor {
        let upstream = (c_plus - floor) / b;
        let downstream = (floor - c_minus) / b;
        let v = volume + dt * (downstream - upstream);
        if v > 0.0 {
            return (floor, upstream, downstream, v);
        }
    }
    let q = (c_plus - c_minus) / (2.0 * b);
    (head, q, q, 0.0)
}

/// A node, given its usual solution (`head`, `outflow`) and how its own
/// outflow depends on its head (`outflow_at`): if it cavitates, the head
/// is held at `floor` and the cavity takes up the imbalance. Returns
/// (head, outflow, cavity volume).
#[must_use]
pub fn node(
    arrivals: &[Arrival],
    (head, outflow): (f64, f64),
    floor: f64,
    volume: f64,
    dt: f64,
    outflow_at: impl Fn(f64) -> f64,
) -> (f64, f64, f64) {
    if volume > 0.0 || head < floor {
        let inflow: f64 = arrivals
            .iter()
            .map(|a| match a.end {
                PipeEnd::Downstream => a.flow(floor),
                PipeEnd::Upstream => -a.flow(floor),
            })
            .sum();
        let out = outflow_at(floor);
        let v = volume + dt * (out - inflow);
        if v > 0.0 {
            return (floor, out, v);
        }
    }
    (head, outflow, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn above_the_floor_nothing_changes() {
        let (h, qu, qd, v) = point(60.0, 40.0, 100.0, -10.0, 0.0, 1e-3);
        assert!((h - 50.0).abs() < 1e-12);
        assert!((qu - 0.1).abs() < 1e-12 && (qu - qd).abs() < 1e-18 && v.abs() < 1e-18);
    }

    #[test]
    fn below_the_floor_a_cavity_opens_and_takes_the_difference() {
        // Usual head would be -30 m; held at -10 m instead.
        let (b, dt) = (100.0, 1e-3);
        let (cp, cm) = (-20.0, -40.0);
        let (h, qu, qd, v) = point(cp, cm, b, -10.0, 0.0, dt);
        assert!((h + 10.0).abs() < 1e-12);
        assert!((qu - (cp + 10.0) / b).abs() < 1e-15, "C+ holds");
        assert!((qd - (-10.0 - cm) / b).abs() < 1e-15, "C- holds");
        assert!((v - dt * (qd - qu)).abs() < 1e-18 && v > 0.0);
    }

    #[test]
    fn a_cavity_the_water_fills_collapses() {
        // Columns closing in fast enough to fill the 1e-6 m^3 cavity.
        let (h, qu, qd, v) = point(80.0, 60.0, 100.0, -10.0, 1e-6, 1e-3);
        assert!(v.abs() < 1e-18);
        assert!((h - 70.0).abs() < 1e-12 && (qu - qd).abs() < 1e-18);
    }
}
