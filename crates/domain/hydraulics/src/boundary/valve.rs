//! Valve at the downstream end of a pipe, discharging to a fixed head.
//!
//! Orifice law, valid for flow in either direction:
//!   Q = tau Q0 sqrt((H - `H_down`) / dH0)
//! where Q0 and dH0 are the steady flow and head drop at full opening
//! (tau = 1). `docs/02_physics_of_transients/governing_equations_and_moc.txt`
//! section 2.

/// The valve coefficient Cv = (tau Q0)^2 / (2 dH0) for the current opening.
///
/// Zero when the valve is shut or never passed flow.
#[must_use]
#[inline]
pub fn coefficient(opening: f64, steady_flow: f64, steady_head_drop: f64) -> f64 {
    if steady_flow == 0.0 {
        return 0.0;
    }
    let tau_q0 = opening * steady_flow;
    tau_q0 * tau_q0 / (2.0 * steady_head_drop)
}

/// Valve at the DOWNSTREAM end: returns (head, flow) at the valve from the
/// arriving C+ value, the head below the valve and the valve coefficient.
#[must_use]
#[inline]
pub fn downstream(c_plus: f64, head_below: f64, impedance: f64, coefficient: f64) -> (f64, f64) {
    let q = flow(c_plus - head_below, impedance, coefficient);
    (c_plus - impedance * q, q)
}

/// Solves Q|Q| = 2 Cv (d - B Q) with d = `C_P` - `H_down`.
///
/// Written in the form without subtractive cancellation,
/// Q = 2 Cv d / (B Cv + sqrt((B Cv)^2 + 2 Cv |d|)),
/// which covers both flow directions.
fn flow(d: f64, b: f64, cv: f64) -> f64 {
    if cv == 0.0 || d == 0.0 {
        return 0.0;
    }
    let bcv = b * cv;
    2.0 * cv * d / (bcv + (bcv * bcv + 2.0 * cv * d.abs()).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_the_orifice_law_both_ways() {
        let (b, cv) = (5000.0, 1e-5);
        for d in [3.0, -3.0, 1e-9, 250.0] {
            let q = flow(d, b, cv);
            let residual = q * q.abs() - 2.0 * cv * (d - b * q);
            assert!(
                residual.abs() <= 1e-12 * 2.0 * cv * d.abs(),
                "d = {d}: residual {residual}"
            );
            assert!(
                q.is_sign_positive() == d.is_sign_positive(),
                "d = {d}: q = {q}"
            );
        }
    }

    #[test]
    fn a_shut_valve_passes_nothing_and_takes_the_full_characteristic_head() {
        let (h, q) = downstream(180.0, 50.0, 5000.0, 0.0);
        assert!(q.abs() < f64::EPSILON);
        assert!((h - 180.0).abs() < f64::EPSILON);
        assert!(coefficient(0.0, 0.07, 50.0).abs() < f64::EPSILON);
        assert!(coefficient(1.0, 0.0, 50.0).abs() < f64::EPSILON);
    }

    #[test]
    fn fully_open_valve_passes_its_steady_flow() {
        // At steady state the C+ value is H_valve + B Q0, so the valve must
        // return exactly Q0 and H_valve.
        let (q0, dh0, hd, b) = (0.07, 50.0, 50.0, 4500.0);
        let h_valve = hd + dh0;
        let (h, q) = downstream(h_valve + b * q0, hd, b, coefficient(1.0, q0, dh0));
        assert!((q - q0).abs() < 1e-15);
        assert!((h - h_valve).abs() < 1e-9);
    }
}
