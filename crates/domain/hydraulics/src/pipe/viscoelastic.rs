//! Plastic pipe walls (PE, PVC): the wall stretches at once under a
//! pressure change, then keeps creeping - viscoelasticity (milestone S6).
//!
//! `docs/02_physics_of_transients/friction_and_damping.txt` section 3;
//! Covas et al. 2004/2005 (J. Hydraulic Research, Parts I and II)
//! [VERIFY equations and parameters against the papers].
//!
//! The instantaneous (elastic) part of the wall's stretch is already in the
//! pipe's wave speed a. The delayed part - the retarded strain - is modelled
//! by up to three Kelvin-Voigt elements, each a spring (creep compliance
//! `J_k` [1/Pa]) beside a dashpot (retardation time `tau_k` [s]):
//!
//! ```text
//!   tau_k d(eps_k)/dt + eps_k = J_k sigma(t),     sigma = c (H - H0)
//!   c = rho g alpha D / (2 e)                      (hoop stress per metre of head)
//! ```
//!
//! alpha D / e: restraint factor x diameter / wall thickness. H0 is the head
//! the wall has settled at (the initial steady state). The creep adds to the
//! continuity equation
//!
//! ```text
//!   dH/dt + (a^2 / (g A)) dQ/dx + (2 a^2 / g) d(eps_r)/dt = 0,   eps_r = sum eps_k
//! ```
//!
//! so along each characteristic (C+ and C-) the head loses the creep rate
//! integrated along the characteristic's path, times 2 a^2 / g.
//!
//! Integration along the PATH matters (FN-021). At a sudden front the wall
//! at the front has only just been loaded and creeps at its fastest rate,
//! J sigma / tau; averaging the rate over a time step at one grid point
//! (as if the load had risen gradually through the step) takes half of it,
//! whatever the step - and the front then shrinks at half the right rate.
//! So the rate is taken where each characteristic STARTS (its foot, at the
//! old time) and where it ARRIVES (the new point), trapezoid-rule:
//!
//! ```text
//!   H_P = C -/+ B Q_P - (K / 2) (rate_foot + rate_P),   K = 2 a^2 dt / g
//!   rate = sum_k (J_k sigma - eps_k) / tau_k              (instantaneous)
//! ```
//!
//! Each element's strain is stepped exactly, with sigma varying linearly
//! over the step (e = exp(-dt / tau), s = tau / dt):
//!
//! ```text
//!   eps_new = e eps_old + J [ (1 - s (1 - e)) sigma_new + (s (1 - e) - e) sigma_old ]
//! ```
//!
//! (exact for linear sigma; for constant sigma, the creep curve
//! J sigma (1 - exp(-t / tau))). With it, `rate_P` is linear in the new head,
//! `rate_P` = d1 (`H_P` - H0) + d0, and the characteristic stays linear:
//!
//! ```text
//!   H_P = C' -/+ B' Q_P,   C' = (C - (K/2) (rate_foot + d0 - d1 H0)) / (1 + (K/2) d1),
//!                          B' = B / (1 + (K/2) d1)
//! ```
//!
//! so every junction, valve and pump in the network solves exactly as for
//! a rigid pipe, with C' and B' for the plastic pipes' ends.

use jk_kernel::{Acceleration, Time, WaveSpeed};

use crate::HydraulicsError;

/// Most Kelvin-Voigt elements per wall.
pub const MAX_ELEMENTS: usize = 3;

/// Density of water [kg/m^3] (docs conventions: rho ~ 998 at 20 C).
const WATER_DENSITY: f64 = 998.0;

/// One Kelvin-Voigt element: a spring beside a dashpot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KelvinVoigt {
    /// Creep compliance J [1/Pa]: how much the element finally stretches
    /// per pascal of hoop stress.
    pub compliance: f64,
    /// Retardation time tau: how long it takes to get there.
    pub retardation: Time,
}

/// A viscoelastic pipe wall.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ViscoelasticWall {
    alpha_d_over_e: f64,
    elements: [KelvinVoigt; MAX_ELEMENTS],
    count: usize,
}

impl ViscoelasticWall {
    /// A wall with restraint x diameter / thickness `alpha_d_over_e` and 1 to
    /// 3 Kelvin-Voigt `elements`.
    ///
    /// # Errors
    /// A non-positive `alpha_d_over_e`, no elements or more than three, or
    /// an element with a negative compliance or non-positive retardation.
    pub fn new(alpha_d_over_e: f64, elements: &[KelvinVoigt]) -> Result<Self, HydraulicsError> {
        if !(alpha_d_over_e.is_finite() && alpha_d_over_e > 0.0) {
            return Err(HydraulicsError::invalid(
                "alpha_d_over_e",
                alpha_d_over_e,
                "finite and > 0",
            ));
        }
        if elements.is_empty() || elements.len() > MAX_ELEMENTS {
            #[allow(clippy::cast_precision_loss)] // a handful
            return Err(HydraulicsError::invalid(
                "Kelvin-Voigt elements",
                elements.len() as f64,
                "1 to 3",
            ));
        }
        for k in elements {
            if !(k.compliance.is_finite() && k.compliance >= 0.0) {
                return Err(HydraulicsError::invalid(
                    "creep compliance",
                    k.compliance,
                    "finite and >= 0",
                ));
            }
            let tau = k.retardation.value();
            if !(tau.is_finite() && tau > 0.0) {
                return Err(HydraulicsError::invalid(
                    "retardation time",
                    tau,
                    "finite and > 0",
                ));
            }
        }
        let mut all = [KelvinVoigt {
            compliance: 0.0,
            retardation: Time::new(1.0),
        }; MAX_ELEMENTS];
        all[..elements.len()].copy_from_slice(elements);
        Ok(Self {
            alpha_d_over_e,
            elements: all,
            count: elements.len(),
        })
    }

    /// The Kelvin-Voigt elements.
    #[must_use]
    pub fn elements(&self) -> &[KelvinVoigt] {
        &self.elements[..self.count]
    }

    /// Restraint factor x diameter / wall thickness.
    #[must_use]
    pub fn alpha_d_over_e(&self) -> f64 {
        self.alpha_d_over_e
    }

    /// Hoop stress per metre of head, c = rho g alpha D / (2 e) [Pa/m].
    #[must_use]
    pub fn stress_per_head(&self, gravity: Acceleration) -> f64 {
        WATER_DENSITY * gravity.value() * self.alpha_d_over_e / 2.0
    }
}

/// The retarded strain of each element at one point of the pipe.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Creep {
    strain: [f64; MAX_ELEMENTS],
}

impl Creep {
    /// Total retarded strain.
    #[must_use]
    pub fn total(&self) -> f64 {
        self.strain.iter().sum()
    }
}

/// The step constants of one wall for one time step (fixed for a run).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CreepStep {
    count: usize,
    /// Per element: compliance J [1/Pa] and 1 / tau [1/s].
    compliance: [f64; MAX_ELEMENTS],
    inv_tau: [f64; MAX_ELEMENTS],
    /// Per element: e = exp(-dt / tau).
    decay: [f64; MAX_ELEMENTS],
    /// Per element: J (1 - s (1 - e)) - weight of the new stress.
    new_weight: [f64; MAX_ELEMENTS],
    /// Per element: J (s (1 - e) - e) - weight of the old stress.
    old_weight: [f64; MAX_ELEMENTS],
    /// Hoop stress per metre of head [Pa/m].
    stress_per_head: f64,
    /// K / 2 = a^2 dt / g [m s].
    half_k: f64,
}

impl CreepStep {
    /// Constants for `wall` in a pipe of wave speed `a`, time step `dt`.
    #[must_use]
    pub fn new(wall: &ViscoelasticWall, a: WaveSpeed, dt: Time, gravity: Acceleration) -> Self {
        let dt_s = dt.value();
        let mut step = Self {
            count: wall.count,
            compliance: [0.0; MAX_ELEMENTS],
            inv_tau: [0.0; MAX_ELEMENTS],
            decay: [0.0; MAX_ELEMENTS],
            new_weight: [0.0; MAX_ELEMENTS],
            old_weight: [0.0; MAX_ELEMENTS],
            stress_per_head: wall.stress_per_head(gravity),
            half_k: a.value() * a.value() * dt_s / gravity.value(),
        };
        for (i, el) in wall.elements().iter().enumerate() {
            let tau = el.retardation.value();
            let e = (-dt_s / tau).exp();
            let s = tau / dt_s;
            step.compliance[i] = el.compliance;
            step.inv_tau[i] = 1.0 / tau;
            step.decay[i] = e;
            step.new_weight[i] = el.compliance * (1.0 - s * (1.0 - e));
            step.old_weight[i] = el.compliance * (s * (1.0 - e) - e);
        }
        step
    }

    /// The creep rate NOW at a point with state `creep` whose head stands
    /// `rise` above the settled head H0: sum (J sigma - eps) / tau [1/s].
    #[must_use]
    #[inline]
    pub fn rate_now(&self, creep: &Creep, rise: f64) -> f64 {
        let stress = self.stress_per_head * rise;
        (0..self.count)
            .map(|i| (self.compliance[i] * stress - creep.strain[i]) * self.inv_tau[i])
            .sum()
    }

    /// The creep rate at the NEW time at a point, as `d1 x rise_new + d0`,
    /// given its state `creep` and `rise_old` at the old time: (d1, d0).
    #[must_use]
    #[inline]
    fn rate_next(&self, creep: &Creep, rise_old: f64) -> (f64, f64) {
        let stress_old = self.stress_per_head * rise_old;
        let (mut d1, mut d0) = (0.0, 0.0);
        for i in 0..self.count {
            // eps_new = e eps_old + nw sigma_new + ow sigma_old, and
            // rate_new = (J sigma_new - eps_new) / tau.
            d1 += (self.compliance[i] - self.new_weight[i]) * self.inv_tau[i];
            d0 -= (self.decay[i] * creep.strain[i] + self.old_weight[i] * stress_old)
                * self.inv_tau[i];
        }
        (d1 * self.stress_per_head, d0)
    }

    /// A characteristic `H = c -/+ b Q` of a rigid pipe arriving at a point
    /// P, with the creep folded in: `foot_rate` is [`Self::rate_now`] where
    /// the characteristic starts (old time); `creep`, `settled` (H0) and
    /// `rise_old` describe P. Returns (C', B') for the same form.
    #[must_use]
    #[inline]
    pub fn fold(
        &self,
        c: f64,
        b: f64,
        foot_rate: f64,
        creep: &Creep,
        settled: f64,
        rise_old: f64,
    ) -> (f64, f64) {
        let (d1, d0) = self.rate_next(creep, rise_old);
        let scale = 1.0 / (1.0 + self.half_k * d1);
        (
            (c - self.half_k * (foot_rate + d0 - d1 * settled)) * scale,
            b * scale,
        )
    }

    /// Advances `creep` over the step, the head having risen from
    /// `rise_old` to `rise_new` above H0 (exact for a linear change).
    #[inline]
    pub fn advance(&self, creep: &mut Creep, rise_old: f64, rise_new: f64) {
        let (s_old, s_new) = (
            self.stress_per_head * rise_old,
            self.stress_per_head * rise_new,
        );
        for i in 0..self.count {
            creep.strain[i] = self.decay[i] * creep.strain[i]
                + self.new_weight[i] * s_new
                + self.old_weight[i] * s_old;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::STANDARD_GRAVITY;

    fn wall(j: f64, tau: f64) -> ViscoelasticWall {
        ViscoelasticWall::new(
            20.0,
            &[KelvinVoigt {
                compliance: j,
                retardation: Time::new(tau),
            }],
        )
        .unwrap()
    }

    #[test]
    fn a_constant_pressure_rise_creeps_along_the_exact_curve() {
        // eps(t) = J sigma (1 - exp(-t / tau)) for a step of 10 m of head.
        let w = wall(1e-10, 0.05);
        let dt = 0.001;
        let step = CreepStep::new(&w, WaveSpeed::new(400.0), Time::new(dt), STANDARD_GRAVITY);
        let sigma = w.stress_per_head(STANDARD_GRAVITY) * 10.0;
        let mut creep = Creep::default();
        step.advance(&mut creep, 10.0, 10.0); // already risen at t = 0
        for n in 2..=200 {
            step.advance(&mut creep, 10.0, 10.0);
            let t = f64::from(n) * dt;
            let exact = 1e-10 * sigma * (1.0 - (-t / 0.05).exp());
            assert!((creep.total() - exact).abs() <= 1e-12 * exact.abs().max(1e-30));
        }
    }

    #[test]
    fn the_new_rate_is_what_advancing_then_gives() {
        let w = wall(3e-10, 0.2);
        let step = CreepStep::new(
            &w,
            WaveSpeed::new(350.0),
            Time::new(0.002),
            STANDARD_GRAVITY,
        );
        let mut creep = Creep::default();
        step.advance(&mut creep, 0.0, 4.0);
        let (d1, d0) = step.rate_next(&creep, 4.0);
        step.advance(&mut creep, 4.0, 7.5);
        let rate = step.rate_now(&creep, 7.5);
        assert!((rate - (d1 * 7.5 + d0)).abs() <= 1e-12 * rate.abs());
    }

    #[test]
    fn without_creep_a_characteristic_is_unchanged() {
        let w = wall(0.0, 0.1);
        let step = CreepStep::new(
            &w,
            WaveSpeed::new(400.0),
            Time::new(0.001),
            STANDARD_GRAVITY,
        );
        let (c, b) = step.fold(55.0, 1234.0, 0.0, &Creep::default(), 50.0, 3.0);
        assert!((c - 55.0).abs() < 1e-12 && (b - 1234.0).abs() < 1e-9);
    }

    #[test]
    fn walls_are_checked() {
        let k = KelvinVoigt {
            compliance: 1e-10,
            retardation: Time::new(0.1),
        };
        assert!(ViscoelasticWall::new(0.0, &[k]).is_err());
        assert!(ViscoelasticWall::new(20.0, &[]).is_err());
        assert!(ViscoelasticWall::new(20.0, &[k; 4]).is_err());
        let bad = KelvinVoigt {
            retardation: Time::ZERO,
            ..k
        };
        assert!(ViscoelasticWall::new(20.0, &[bad]).is_err());
    }
}
