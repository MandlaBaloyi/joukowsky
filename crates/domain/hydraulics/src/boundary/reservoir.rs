//! Reservoir: a boundary that holds the head fixed, whatever the flow.
//!
//! A large reservoir reflects a pressure wave inverted (R = -1,
//! `docs/02_physics_of_transients/joukowsky_equation.txt` section 6).

/// Reservoir at the UPSTREAM end of a pipe.
///
/// Returns (head, flow) from the arriving C- value: Q = (`H_res` - `C_M`) / B.
#[must_use]
#[inline]
pub fn upstream(reservoir_head: f64, c_minus: f64, impedance: f64) -> (f64, f64) {
    (reservoir_head, (reservoir_head - c_minus) / impedance)
}

/// Reservoir at the DOWNSTREAM end of a pipe.
///
/// Returns (head, flow) from the arriving C+ value: Q = (`C_P` - `H_res`) / B.
#[must_use]
#[inline]
pub fn downstream(reservoir_head: f64, c_plus: f64, impedance: f64) -> (f64, f64) {
    (reservoir_head, (c_plus - reservoir_head) / impedance)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn head_is_held_and_flow_satisfies_the_characteristic() {
        let (hr, b) = (100.0, 5000.0);
        let (h, q) = upstream(hr, 90.0, b);
        assert!((h - hr).abs() < f64::EPSILON);
        assert!((h - (90.0 + b * q)).abs() < 1e-12, "C- not satisfied");
        let (h, q) = downstream(hr, 110.0, b);
        assert!((h - hr).abs() < f64::EPSILON);
        assert!((h - (110.0 - b * q)).abs() < 1e-12, "C+ not satisfied");
    }
}
