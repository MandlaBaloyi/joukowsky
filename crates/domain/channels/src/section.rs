//! Cross-sections and reaches (`docs/02_physics_of_transients/
//! open_channels.txt`, section 1).
//!
//! A section is a trapezoid: bottom width b, side slope m (horizontal per
//! vertical; 0 is a rectangle). Everything the equations need is a closed
//! formula in the depth h:
//!
//! ```text
//!   A = (b + m h) h      T = b + 2 m h      P = b + 2 h sqrt(1 + m^2)
//!   I1 = b h^2 / 2 + m h^3 / 3      (hydrostatic pressure term)
//! ```

use jk_kernel::{Acceleration, Flow, Length};

use crate::error::{ChannelError, positive};

/// A trapezoidal (or rectangular) cross-section.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Section {
    /// Bottom width [m] (> 0).
    pub bottom_width: Length,
    /// Side slope, horizontal per vertical (0: vertical walls).
    pub side_slope: f64,
}

impl Section {
    /// A rectangle of width `width`.
    #[must_use]
    pub fn rectangle(width: Length) -> Self {
        Self {
            bottom_width: width,
            side_slope: 0.0,
        }
    }

    /// Checks the dimensions.
    ///
    /// # Errors
    /// A bottom width that is not positive, or a negative side slope.
    pub fn validate(&self) -> Result<(), ChannelError> {
        positive("bottom width", self.bottom_width.value())?;
        if !(self.side_slope.is_finite() && self.side_slope >= 0.0) {
            return Err(ChannelError::InvalidInput(format!(
                "side slope must be >= 0, got {}",
                self.side_slope
            )));
        }
        Ok(())
    }

    /// Wetted area at depth `h` [m^2].
    #[must_use]
    #[inline]
    pub fn area(&self, h: f64) -> f64 {
        (self.bottom_width.value() + self.side_slope * h) * h
    }

    /// Depth at wetted area `a` [m].
    #[must_use]
    #[inline]
    pub fn depth(&self, a: f64) -> f64 {
        let (b, m) = (self.bottom_width.value(), self.side_slope);
        if a <= 0.0 {
            0.0
        } else if m == 0.0 {
            a / b
        } else {
            // m h^2 + b h - a = 0, the stable root.
            2.0 * a / (b + (b * b + 4.0 * m * a).sqrt())
        }
    }

    /// Top width at depth `h` [m].
    #[must_use]
    #[inline]
    pub fn top_width(&self, h: f64) -> f64 {
        self.bottom_width.value() + 2.0 * self.side_slope * h
    }

    /// Wetted perimeter at depth `h` [m].
    #[must_use]
    #[inline]
    pub fn perimeter(&self, h: f64) -> f64 {
        self.bottom_width.value() + 2.0 * h * (1.0 + self.side_slope * self.side_slope).sqrt()
    }

    /// Hydrostatic pressure term I1 at depth `h` [m^3].
    #[must_use]
    #[inline]
    pub fn pressure_term(&self, h: f64) -> f64 {
        let b = self.bottom_width.value();
        b * h * h / 2.0 + self.side_slope * h * h * h / 3.0
    }

    /// Small-wave celerity sqrt(g A / T) at depth `h` [m/s].
    #[must_use]
    #[inline]
    pub fn celerity(&self, h: f64, g: f64) -> f64 {
        if h <= 0.0 {
            0.0
        } else {
            (g * self.area(h) / self.top_width(h)).sqrt()
        }
    }
}

/// A reach: a stretch of channel of one section, slope and roughness.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reach {
    /// Length along the channel [m].
    pub length: Length,
    /// The cross-section.
    pub section: Section,
    /// Bed slope (fall per length; positive downhill in the flow
    /// direction, 0 flat).
    pub slope: f64,
    /// Manning's roughness n [s/m^(1/3)] (concrete-lined canal about
    /// 0.013-0.017, earth channel 0.022-0.030 [VERIFY per channel]).
    pub manning: f64,
    /// Gravity.
    pub gravity: Acceleration,
}

impl Reach {
    /// Checks the reach.
    ///
    /// # Errors
    /// A length or Manning n that is not positive, a slope that is not
    /// finite, or a bad section.
    pub fn validate(&self) -> Result<(), ChannelError> {
        positive("reach length", self.length.value())?;
        if !(self.manning.is_finite() && self.manning >= 0.0) {
            return Err(ChannelError::InvalidInput(format!(
                "Manning n must be >= 0, got {}",
                self.manning
            )));
        }
        if !self.slope.is_finite() {
            return Err(ChannelError::InvalidInput(
                "bed slope must be finite".into(),
            ));
        }
        positive("gravity", self.gravity.value())?;
        self.section.validate()
    }

    /// Friction slope Sf = n^2 Q|Q| / (A^2 R^(4/3)) at depth `h`, flow `q`.
    #[must_use]
    #[inline]
    pub fn friction_slope(&self, h: f64, q: f64) -> f64 {
        let a = self.section.area(h);
        if a <= 0.0 {
            return 0.0;
        }
        let r = a / self.section.perimeter(h);
        self.manning * self.manning * q * q.abs() / (a * a * r.powf(4.0 / 3.0))
    }

    /// Normal depth: the depth of uniform flow carrying `flow`, where
    /// friction balances the bed slope (Manning).
    ///
    /// # Errors
    /// A bed that is flat or rises (no uniform flow), or a flow that is
    /// not positive.
    pub fn normal_depth(&self, flow: Flow) -> Result<Length, ChannelError> {
        self.validate()?;
        let q = positive("flow", flow.value())?;
        if self.slope <= 0.0 || self.manning <= 0.0 {
            return Err(ChannelError::InvalidInput(
                "uniform flow needs a downhill bed and friction".into(),
            ));
        }
        // Sf falls as h rises: bisection on Sf(h) - S0.
        let h = bisect(|h| self.slope - self.friction_slope(h, q));
        Ok(Length::new(h))
    }

    /// Critical depth: where the flow is exactly as fast as a small wave
    /// (Froude 1), Q^2 T / (g A^3) = 1.
    ///
    /// # Errors
    /// A flow that is not positive, or a bad reach.
    pub fn critical_depth(&self, flow: Flow) -> Result<Length, ChannelError> {
        self.validate()?;
        let q = positive("flow", flow.value())?;
        let g = self.gravity.value();
        let s = &self.section;
        let h = bisect(|h| {
            let a = s.area(h);
            g * a * a * a - q * q * s.top_width(h)
        });
        Ok(Length::new(h))
    }
}

/// The root of an increasing function f with f(0+) < 0, by bracketing then
/// bisection to machine precision.
fn bisect(f: impl Fn(f64) -> f64) -> f64 {
    let (mut lo, mut hi) = (1e-9, 1.0);
    while f(hi) < 0.0 && hi < 1e6 {
        lo = hi;
        hi *= 2.0;
    }
    for _ in 0..200 {
        let mid = f64::midpoint(lo, hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo <= f64::EPSILON * hi {
            break;
        }
    }
    f64::midpoint(lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::STANDARD_GRAVITY;

    #[test]
    fn depth_and_area_are_inverse() {
        let s = Section {
            bottom_width: Length::new(3.0),
            side_slope: 1.5,
        };
        for h in [0.01, 0.5, 1.7, 4.0] {
            assert!((s.depth(s.area(h)) - h).abs() < 1e-12);
        }
        let r = Section::rectangle(Length::new(2.0));
        assert!((r.depth(r.area(1.3)) - 1.3).abs() < 1e-15);
    }

    #[test]
    fn critical_flow_has_froude_one() {
        let reach = Reach {
            length: Length::new(1000.0),
            section: Section::rectangle(Length::new(2.0)),
            slope: 0.001,
            manning: 0.015,
            gravity: STANDARD_GRAVITY,
        };
        let q = 3.0;
        let hc = reach.critical_depth(Flow::new(q)).unwrap().value();
        // Rectangle: hc = (q_unit^2 / g)^(1/3), q_unit = Q / b.
        let expected = ((q / 2.0).powi(2) / STANDARD_GRAVITY.value()).cbrt();
        assert!((hc - expected).abs() < 1e-12);
    }
}
