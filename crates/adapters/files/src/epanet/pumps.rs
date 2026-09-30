//! EPANET pumps: their head curves fitted to the engine's pump curve
//! (`crates/domain/hydraulics/src/boundary/pump.rs`: H = a0 + a1 Q +
//! a2 Q^2), so a pump in a file pumps in steady states and extended-period
//! runs, and trips in transients.
//!
//! EPANET's own curve forms (EPANET 2.2 manual, "Pump curves"):
//! - one point (Q1, H1): H = 4/3 H1 - (H1/3) (Q/Q1)^2, a quadratic - taken
//!   exactly;
//! - three points, the first at zero flow: a power law H = A - B Q^C
//!   through them. The engine takes the quadratic through the same three
//!   points; the note says how far apart the two are between the points;
//! - any other number of points: straight lines between the points. The
//!   engine takes the least-squares quadratic; the note gives its largest
//!   miss at the points.
//!
//! Only transients need what files do not give - efficiency, speed and
//! rotating inertia (for the run-down after a trip) - so these are assumed
//! and said so.

use std::collections::HashMap;

use jk_hydraulics::boundary::pump::WATER_DENSITY;
use jk_hydraulics::{Pump, PumpCurve, thorley_inertia};
use jk_kernel::Flow;

/// Efficiency assumed at the rated point (files give efficiency curves only
/// for energy costs, rarely).
pub const ASSUMED_EFFICIENCY: f64 = 0.75;
/// Speed assumed [rpm] (a 4-pole motor on 50 Hz).
pub const ASSUMED_SPEED_RPM: f64 = 1450.0;

/// A pump fitted from its curve: the pump and words on how it was fitted.
pub struct Fitted {
    /// The pump, curve fitted and the rest assumed.
    pub pump: Pump,
    /// How the curve was fitted, in words (for a note).
    pub how: String,
}

/// Fits a pump to curve points (flow [m^3/s], head [m], in SI) at relative
/// speed `speed` (affinity laws).
///
/// # Errors
/// Points that do not make a pump: flows not rising, heads not falling, or
/// a fit that does not droop.
pub fn fit(points: &[(f64, f64)], speed: f64) -> Result<Fitted, String> {
    if points.is_empty() {
        return Err("its curve has no points".into());
    }
    if points
        .windows(2)
        .any(|w| !(w[1].0 > w[0].0 && w[1].1 < w[0].1))
        || points.iter().any(|p| !(p.0 >= 0.0 && p.1 > 0.0))
    {
        return Err("its curve's flows must rise and heads fall, all positive".into());
    }
    let (a0, a1, a2, rated, how) = match points {
        [(q1, h1)] => {
            if *q1 <= 0.0 {
                return Err("a one-point curve needs a flow above zero".into());
            }
            (
                4.0 / 3.0 * h1,
                0.0,
                -h1 / (3.0 * q1 * q1),
                *q1,
                "one-point curve, taken exactly (EPANET's own form)".to_string(),
            )
        }
        [(q0, h0), (q1, h1), (q2, h2)] if *q0 == 0.0 => {
            let c = PumpCurve::from_points(
                jk_kernel::Head::new(*h0),
                (Flow::new(*q1), jk_kernel::Head::new(*h1)),
                (Flow::new(*q2), jk_kernel::Head::new(*h2)),
            )
            .map_err(|e| e.to_string())?;
            let (a0, a1, a2) = c.coefficients();
            // EPANET's power law through the same points.
            let power = ((h0 - h2) / (h0 - h1)).ln() / (q2 / q1).ln();
            let b = (h0 - h1) / q1.powf(power);
            let worst = (0..=40)
                .map(|k| {
                    let q = q2 * f64::from(k) / 40.0;
                    ((a0 + a1 * q + a2 * q * q) - (h0 - b * q.powf(power))).abs()
                })
                .fold(0.0, f64::max);
            (
                a0,
                a1,
                a2,
                *q1,
                format!(
                    "three-point curve: a quadratic through the points (EPANET's power law \
                     H = A - B Q^{power:.2} differs by up to {worst:.2} m between them)"
                ),
            )
        }
        _ => {
            let (a0, a1, a2) = least_squares(points);
            let worst = points
                .iter()
                .map(|(q, h)| (a0 + a1 * q + a2 * q * q - h).abs())
                .fold(0.0, f64::max);
            (
                a0,
                a1,
                a2,
                points[points.len() / 2]
                    .0
                    .max(points[points.len() - 1].0 / 2.0),
                format!(
                    "{}-point curve: least-squares quadratic (EPANET draws straight lines \
                     between the points; largest miss at a point {worst:.2} m)",
                    points.len()
                ),
            )
        }
    };
    // Affinity laws at relative speed s: H(Q) = a0 s^2 + a1 s Q + a2 Q^2.
    let curve = PumpCurve::from_coefficients(a0 * speed * speed, a1 * speed, a2)
        .map_err(|e| format!("its fitted curve is not a pump curve ({e})"))?;
    let rated_flow = rated * speed;
    let rated_head = curve.head(rated_flow, 1.0);
    if rated_head <= 0.0 {
        return Err("its fitted curve gives no head at the rated flow".into());
    }
    let power_kw = WATER_DENSITY * 9.81 * rated_flow * rated_head / ASSUMED_EFFICIENCY / 1000.0;
    Ok(Fitted {
        pump: Pump {
            curve,
            rated_flow: Flow::new(rated_flow),
            rated_efficiency: ASSUMED_EFFICIENCY,
            rated_speed_rpm: ASSUMED_SPEED_RPM,
            inertia: thorley_inertia(power_kw, ASSUMED_SPEED_RPM),
        },
        how,
    })
}

/// Least-squares a0 + a1 Q + a2 Q^2 through the points (with two points:
/// a0 + a2 Q^2 exactly through both).
fn least_squares(points: &[(f64, f64)]) -> (f64, f64, f64) {
    if let [(q1, h1), (q2, h2)] = points {
        let a2 = (h2 - h1) / (q2 * q2 - q1 * q1);
        return (h1 - a2 * q1 * q1, 0.0, a2);
    }
    // Normal equations, scaled by the largest flow for conditioning.
    let scale = points.iter().map(|p| p.0).fold(0.0, f64::max).max(1e-12);
    let mut m = [[0.0_f64; 4]; 3];
    for &(q, h) in points {
        let x = q / scale;
        let basis = [1.0, x, x * x];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] += basis[i] * basis[j];
            }
            m[i][3] += basis[i] * h;
        }
    }
    // Gaussian elimination with partial pivoting.
    for col in 0..3 {
        let pivot = (col..3)
            .max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))
            .unwrap_or(col);
        m.swap(col, pivot);
        for row in 0..3 {
            if row != col && m[col][col] != 0.0 {
                let f = m[row][col] / m[col][col];
                let pivot_row = m[col];
                for (v, p) in m[row].iter_mut().zip(pivot_row).skip(col) {
                    *v -= f * p;
                }
            }
        }
    }
    let c: Vec<f64> = (0..3).map(|i| m[i][3] / m[i][i]).collect();
    (c[0], c[1] / scale, c[2] / (scale * scale))
}

/// Pump curves in `[CURVES]`, by ID, in file units: (x, y) points in file
/// order.
pub type Curves = HashMap<String, Vec<(f64, f64)>>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_point_curve_is_epanets_own() {
        let f = fit(&[(0.1, 60.0)], 1.0).unwrap();
        let c = f.pump.curve;
        assert!((c.head(0.0, 1.0) - 80.0).abs() < 1e-12);
        assert!((c.head(0.1, 1.0) - 60.0).abs() < 1e-12);
        assert!(c.head(0.2, 1.0).abs() < 1e-9); // run-out at twice the flow
    }

    #[test]
    fn three_points_are_matched_exactly() {
        let pts = [(0.0, 80.0), (0.1, 65.0), (0.15, 45.0)];
        let c = fit(&pts, 1.0).unwrap().pump.curve;
        for (q, h) in pts {
            assert!((c.head(q, 1.0) - h).abs() < 1e-9);
        }
    }

    #[test]
    fn many_points_least_squares_recovers_a_quadratic() {
        let h = |q: f64| 50.0 + 20.0 * q - 900.0 * q * q;
        let pts: Vec<(f64, f64)> = (0..6)
            .map(|k| f64::from(k) * 0.04)
            .map(|q| (q, h(q)))
            .collect();
        let c = fit(&pts, 1.0).unwrap().pump.curve;
        let (a0, a1, a2) = c.coefficients();
        assert!((a0 - 50.0).abs() < 1e-8 && (a1 - 20.0).abs() < 1e-6 && (a2 + 900.0).abs() < 1e-5);
    }

    #[test]
    fn speed_scales_by_the_affinity_laws() {
        let c = fit(&[(0.1, 60.0)], 0.5).unwrap().pump.curve;
        assert!((c.head(0.0, 1.0) - 20.0).abs() < 1e-12); // 80 x 0.25
    }

    #[test]
    fn rising_heads_are_not_a_pump() {
        assert!(fit(&[(0.0, 10.0), (0.1, 20.0), (0.2, 5.0)], 1.0).is_err());
    }
}
