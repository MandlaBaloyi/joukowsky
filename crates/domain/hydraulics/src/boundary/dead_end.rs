//! Dead end (closed end): no flow can pass, so a wave is reflected whole
//! and the pressure DOUBLES (R = +1, `docs/02_physics_of_transients/
//! joukowsky_equation.txt` section 6).
//!
//! Dead ends are everywhere in real networks - closed boundary valves,
//! capped branches, unused connections - and they are where surges are
//! amplified most. A shut valve behaves exactly like one.

/// Dead end at the DOWNSTREAM end of a pipe: Q = 0, so H = `C_P`.
#[must_use]
#[inline]
pub fn downstream(c_plus: f64) -> (f64, f64) {
    (c_plus, 0.0)
}

/// Dead end at the UPSTREAM end of a pipe: Q = 0, so H = `C_M`.
#[must_use]
#[inline]
pub fn upstream(c_minus: f64) -> (f64, f64) {
    (c_minus, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_flow_and_head_from_the_characteristic() {
        assert_eq!(downstream(123.0), (123.0, 0.0));
        assert_eq!(upstream(77.0), (77.0, 0.0));
    }
}
