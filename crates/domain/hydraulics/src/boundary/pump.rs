//! Centrifugal pump with its discharge non-return valve: normal running,
//! and run-down after a power failure (a "pump trip").
//!
//! `docs/01_problem_definition/the_operating_environment.txt` section 1 and
//! `docs/06_engine_architecture/outages_scale_and_errors.txt` 1a-1b: in South
//! Africa load-shedding trips pumps routinely. When power is cut the pump
//! slows under the torque of the water it is still pushing, resisted only by
//! its rotating inertia (Newton's second law for rotation):
//!
//! ```text
//!   I * w_R * d(alpha)/dt = -T        alpha = N / N_R  (speed ratio)
//! ```
//!
//! Head and torque follow the affinity (homologous) laws from the rated
//! curve - valid in the normal operating zone (forward flow, forward speed):
//!
//! ```text
//!   head      H = a0 alpha^2 + a1 alpha Q + a2 Q^2
//!   torque    T = rho g Q H / (eta w)
//!   efficiency eta(q) = eta_R (2 q/Q_R - (q/Q_R)^2),   q = Q / alpha
//! ```
//!
//! With that parabolic efficiency (zero at zero flow, peak `eta_R` at the
//! rated flow `Q_R`) the torque simplifies to
//! T = rho g H `Q_R` / (`w_R` `eta_R` (2 - q/`Q_R`)) - finite at zero flow, as
//! real shut-off torque is.
//!
//! SCOPE (version 1): the pump has a non-return valve on its discharge, the
//! standard pump-station layout, so flow through it never reverses and the
//! normal zone is all that is needed. Pumps WITHOUT a non-return valve can
//! reverse and turbine, which needs four-quadrant (Suter) curves - version 2,
//! together with the dynamic valve slam. The ideal non-return valve here
//! shuts the instant flow would reverse.

use jk_kernel::{Flow, Head};

use crate::HydraulicsError;
use crate::error::positive;

/// Water density at 20 C [kg/m^3]. The fluid module (milestone S7) will
/// make this temperature-dependent.
pub const WATER_DENSITY: f64 = 998.2;

/// Head-flow curve at rated speed: H = a0 + a1 Q + a2 Q^2.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpCurve {
    a0: f64,
    a1: f64,
    a2: f64,
}

impl PumpCurve {
    /// Fits the curve through three datasheet points: the shut-off head
    /// (zero flow), the rated point and a run-out point at higher flow.
    ///
    /// # Errors
    /// Non-physical points: flows must rise and heads fall, and the curve
    /// must droop (a2 < 0) so the pump has a stable operating point.
    pub fn from_points(
        shutoff: Head,
        rated: (Flow, Head),
        runout: (Flow, Head),
    ) -> Result<Self, HydraulicsError> {
        let (h0, (q1, h1), (q2, h2)) = (
            shutoff.value(),
            (rated.0.value(), rated.1.value()),
            (runout.0.value(), runout.1.value()),
        );
        positive("rated flow", q1)?;
        if !(q2 > q1 && h0 > h1 && h1 > h2 && h2 > 0.0 && h0.is_finite() && q2.is_finite()) {
            return Err(HydraulicsError::invalid(
                "pump curve",
                h0,
                "0 < Q_rated < Q_runout with shut-off head > rated head > run-out head > 0",
            ));
        }
        // Solve a1 q + a2 q^2 = h - h0 at the two points.
        let (d1, d2) = (h1 - h0, h2 - h0);
        let a2 = (d2 / q2 - d1 / q1) / (q2 - q1);
        let a1 = d1 / q1 - a2 * q1;
        if a2 >= 0.0 {
            return Err(HydraulicsError::invalid(
                "pump curve a2",
                a2,
                "< 0 (a drooping curve)",
            ));
        }
        Ok(Self { a0: h0, a1, a2 })
    }

    /// A curve from its coefficients, H = a0 + a1 Q + a2 Q^2 (e.g. fitted
    /// to a curve in a network file).
    ///
    /// # Errors
    /// A shut-off head a0 that is not positive, non-finite coefficients, or
    /// a curve that does not droop (a2 >= 0).
    pub fn from_coefficients(a0: f64, a1: f64, a2: f64) -> Result<Self, HydraulicsError> {
        positive("pump shut-off head a0", a0)?;
        if !(a1.is_finite() && a2.is_finite() && a2 < 0.0) {
            return Err(HydraulicsError::invalid(
                "pump curve a2",
                a2,
                "< 0 (a drooping curve), a1 finite",
            ));
        }
        Ok(Self { a0, a1, a2 })
    }

    /// Head added at flow `q` [m^3/s] and speed ratio `alpha` (affinity laws).
    #[must_use]
    #[inline]
    pub fn head(&self, q: f64, alpha: f64) -> f64 {
        self.a0 * alpha * alpha + self.a1 * alpha * q + self.a2 * q * q
    }

    /// Slope dH/dQ at flow `q` and speed ratio `alpha`.
    #[must_use]
    #[inline]
    pub fn slope(&self, q: f64, alpha: f64) -> f64 {
        self.a1 * alpha + 2.0 * self.a2 * q
    }

    /// The three coefficients (a0, a1, a2).
    #[must_use]
    pub fn coefficients(&self) -> (f64, f64, f64) {
        (self.a0, self.a1, self.a2)
    }
}

/// A pump and its drive: curve, rated point, rotating inertia.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pump {
    /// Head-flow curve at rated speed.
    pub curve: PumpCurve,
    /// Flow at the best-efficiency (rated) point.
    pub rated_flow: Flow,
    /// Efficiency at the rated point (0..1).
    pub rated_efficiency: f64,
    /// Rated speed [rpm].
    pub rated_speed_rpm: f64,
    /// Rotating inertia of pump + motor + entrained water [kg m^2]. Use
    /// [`thorley_inertia`] if the datasheet does not give it.
    pub inertia: f64,
}

impl Pump {
    /// Checks the pump's numbers.
    ///
    /// # Errors
    /// Non-positive rated flow, speed or inertia, or an efficiency outside
    /// (0, 1].
    pub fn validate(&self) -> Result<(), HydraulicsError> {
        positive("pump rated_flow", self.rated_flow.value())?;
        positive("pump rated_speed_rpm", self.rated_speed_rpm)?;
        positive("pump inertia", self.inertia)?;
        if !(self.rated_efficiency > 0.0 && self.rated_efficiency <= 1.0) {
            return Err(HydraulicsError::invalid(
                "pump rated_efficiency",
                self.rated_efficiency,
                "in (0, 1]",
            ));
        }
        Ok(())
    }

    /// Rated angular speed `w_R` [rad/s].
    #[must_use]
    pub fn rated_omega(&self) -> f64 {
        self.rated_speed_rpm * std::f64::consts::TAU / 60.0
    }

    /// Hydraulic torque [N m] at flow `q` and speed ratio `alpha`, with
    /// gravity `g`: T = rho g H `Q_R` / (`w_R` `eta_R` (2 - q/`Q_R`)),
    /// q = Q/alpha (clamped so the efficiency parabola stays positive).
    #[must_use]
    pub fn torque(&self, q: f64, alpha: f64, g: f64) -> f64 {
        let qr = self.rated_flow.value();
        let head = self.curve.head(q, alpha);
        let relative = if alpha > 0.0 { (q / alpha) / qr } else { 0.0 };
        let relative = relative.clamp(0.0, 1.9);
        WATER_DENSITY * g * head * qr
            / (self.rated_omega() * self.rated_efficiency * (2.0 - relative))
    }

    /// Rate of change of the speed ratio after a trip: -T / (I `w_R`).
    #[must_use]
    pub fn deceleration(&self, q: f64, alpha: f64, g: f64) -> f64 {
        -self.torque(q, alpha, g) / (self.inertia * self.rated_omega())
    }
}

/// Estimated rotating inertia of a pump and its motor [kg m^2] from rated
/// shaft power `power_kw` [kW] and speed `speed_rpm` [rpm] (Thorley's
/// regressions over 284 pumps and 272 motors):
///
/// ```text
///   I_pump  = 1.5e7 (P / N^3)^0.9556
///   I_motor = 118   (P / N)^1.48
/// ```
///
/// Real values scatter from -50% to +100% around these, so treat the result
/// as a prior with wide uncertainty (principle P3), not a fact.
#[must_use]
pub fn thorley_inertia(power_kw: f64, speed_rpm: f64) -> f64 {
    let pump = 1.5e7 * (power_kw / speed_rpm.powi(3)).powf(0.9556);
    let motor = 118.0 * (power_kw / speed_rpm).powf(1.48);
    pump + motor
}

/// Heads on both sides of the pump, its flow, and whether it passes flow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PumpStep {
    /// Head at the suction side [m].
    pub head_suction: f64,
    /// Head at the discharge side [m].
    pub head_discharge: f64,
    /// Flow through the pump (>= 0) [m^3/s].
    pub flow: f64,
    /// False when the non-return valve is shut.
    pub flowing: bool,
}

/// Pump between a suction pipe (bringing `C_P`, impedance `b_s`) and a
/// discharge pipe (bringing `C_M`, impedance `b_d`) at speed ratio `alpha`.
///
/// Suction: `H_s` = `C_P` - `b_s` Q; discharge: `H_d` = `C_M` + `b_d` Q; pump:
/// `H_d` - `H_s` = H(Q, alpha). Together a quadratic in Q; the non-return
/// valve takes over (Q = 0, both sides dead ends) if the flow would reverse.
#[must_use]
pub fn step(
    curve: &PumpCurve,
    alpha: f64,
    c_plus_s: f64,
    b_s: f64,
    c_minus_d: f64,
    b_d: f64,
) -> PumpStep {
    let (a0, a1, a2) = curve.coefficients();
    // (-a2) Q^2 + (b_s + b_d - a1 alpha) Q - (a0 alpha^2 + C_P - C_M) = 0
    let a = -a2;
    let b = b_s + b_d - a1 * alpha;
    let c = a0 * alpha * alpha + c_plus_s - c_minus_d;
    let shut = PumpStep {
        head_suction: c_plus_s,
        head_discharge: c_minus_d,
        flow: 0.0,
        flowing: false,
    };
    if c <= 0.0 {
        return shut; // the pump cannot push against the discharge side
    }
    // Positive root, in the form without cancellation.
    let q = 2.0 * c / (b + (b * b + 4.0 * a * c).sqrt());
    PumpStep {
        head_suction: c_plus_s - b_s * q,
        head_discharge: c_minus_d + b_d * q,
        flow: q,
        flowing: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curve() -> PumpCurve {
        PumpCurve::from_points(
            Head::new(80.0),
            (Flow::new(0.10), Head::new(65.0)),
            (Flow::new(0.15), Head::new(45.0)),
        )
        .unwrap()
    }

    #[test]
    fn curve_passes_through_its_three_points_and_scales_with_speed() {
        let c = curve();
        assert!((c.head(0.0, 1.0) - 80.0).abs() < 1e-12);
        assert!((c.head(0.10, 1.0) - 65.0).abs() < 1e-12);
        assert!((c.head(0.15, 1.0) - 45.0).abs() < 1e-12);
        // Affinity laws: at half speed, half the flow gives a quarter of the head.
        assert!((c.head(0.05, 0.5) - 65.0 / 4.0).abs() < 1e-12);
    }

    #[test]
    fn rejects_curves_that_rise_or_cross() {
        let bad = PumpCurve::from_points(
            Head::new(60.0),
            (Flow::new(0.1), Head::new(65.0)),
            (Flow::new(0.15), Head::new(45.0)),
        );
        assert!(bad.is_err());
    }

    #[test]
    fn step_satisfies_both_characteristics_and_the_pump_curve() {
        let c = curve();
        let (cp, bs, cm, bd) = (12.0, 1500.0, 40.0, 1400.0);
        let s = step(&c, 0.9, cp, bs, cm, bd);
        assert!(s.flowing && s.flow > 0.0);
        assert!((s.head_suction - (cp - bs * s.flow)).abs() < 1e-9);
        assert!((s.head_discharge - (cm + bd * s.flow)).abs() < 1e-9);
        let lift = s.head_discharge - s.head_suction;
        assert!((lift - c.head(s.flow, 0.9)).abs() < 1e-9);
    }

    #[test]
    fn non_return_valve_shuts_when_the_pump_cannot_push() {
        let s = step(&curve(), 0.3, 10.0, 1500.0, 60.0, 1400.0);
        assert!(!s.flowing);
        assert_eq!(
            (s.flow, s.head_suction, s.head_discharge),
            (0.0, 10.0, 60.0)
        );
    }

    #[test]
    fn thorley_rpm_form_matches_the_thousand_rpm_form() {
        // The same regressions written with N in 1000s of rpm:
        // I_pump = 0.03768 (P/N^3)^0.9556, I_motor = 0.0043 (P/N)^1.48.
        let (p, n) = (100.0_f64, 1450.0_f64);
        let k = n / 1000.0;
        let other = 0.03768 * (p / k.powi(3)).powf(0.9556) + 0.0043 * (p / k).powf(1.48);
        assert!((thorley_inertia(p, n) - other).abs() / other < 0.01);
    }

    #[test]
    fn rated_torque_is_power_over_speed() {
        let pump = Pump {
            curve: curve(),
            rated_flow: Flow::new(0.10),
            rated_efficiency: 0.8,
            rated_speed_rpm: 1450.0,
            inertia: 2.5,
        };
        let g = 9.81;
        let power = WATER_DENSITY * g * 0.10 * 65.0 / 0.8;
        assert!((pump.torque(0.10, 1.0, g) - power / pump.rated_omega()).abs() < 1e-9);
    }
}
