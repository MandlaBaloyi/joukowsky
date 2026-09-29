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
//! so along both characteristics (C+ and C-) a term K d(`eps_r`)/dt,
//! K = 2 a^2 dt / g, is subtracted from the head at the new time.
//!
//! Stepping one element exactly, with sigma varying linearly over the step
//! (e = exp(-dt / tau), s = tau / dt):
//!
//! ```text
//!   eps_new = e eps_old + J [ (1 - s (1 - e)) sigma_new + (s (1 - e) - e) sigma_old ]
//! ```
//!
//! (exact solution of the element's equation for linear sigma; for a
//! constant sigma it is the creep curve J sigma (1 - exp(-t / tau))). The
//! creep rate at the new time, (`eps_new` - `eps_old`) / dt, is therefore linear
//! in the new head: rate = c1 (H - H0) + c0. Folded into a characteristic
//! H = C -/+ B Q - K rate, it gives an ordinary characteristic
//!
//! ```text
//!   H = C' -/+ B' Q,   C' = (C + K (c1 H0 - c0)) / (1 + K c1),   B' = B / (1 + K c1)
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
    /// Per element: e = exp(-dt / tau).
    decay: [f64; MAX_ELEMENTS],
    /// Per element: J (1 - s (1 - e)) - weight of the new stress.
    new_weight: [f64; MAX_ELEMENTS],
    /// Per element: J (s (1 - e) - e) - weight of the old stress.
    old_weight: [f64; MAX_ELEMENTS],
    /// Hoop stress per metre of head [Pa/m].
    stress_per_head: f64,
    /// K = 2 a^2 dt / g [m s].
    k: f64,
    dt: f64,
}

impl CreepStep {
    /// Constants for `wall` in a pipe of wave speed `a`, time step `dt`.
    #[must_use]
    pub fn new(wall: &ViscoelasticWall, a: WaveSpeed, dt: Time, gravity: Acceleration) -> Self {
        let dt_s = dt.value();
        let mut decay = [0.0; MAX_ELEMENTS];
        let mut new_weight = [0.0; MAX_ELEMENTS];
        let mut old_weight = [0.0; MAX_ELEMENTS];
        for (i, el) in wall.elements().iter().enumerate() {
            let e = (-dt_s / el.retardation.value()).exp();
            let s = el.retardation.value() / dt_s;
            decay[i] = e;
            new_weight[i] = el.compliance * (1.0 - s * (1.0 - e));
            old_weight[i] = el.compliance * (s * (1.0 - e) - e);
        }
        Self {
            count: wall.count,
            decay,
            new_weight,
            old_weight,
            stress_per_head: wall.stress_per_head(gravity),
            k: 2.0 * a.value() * a.value() * dt_s / gravity.value(),
            dt: dt_s,
        }
    }

    /// The creep rate at the new time as `c1 x rise_new + c0` [1/s], for a
    /// point with creep state `creep` whose head stood `rise_old` above the
    /// settled head H0 at the old time: returns (c1 [1/(m s)], c0 [1/s]).
    #[must_use]
    #[inline]
    pub fn rate(&self, creep: &Creep, rise_old: f64) -> (f64, f64) {
        let (mut c1, mut c0) = (0.0, 0.0);
        let stress_old = self.stress_per_head * rise_old;
        for i in 0..self.count {
            c1 += self.new_weight[i];
            c0 += (self.decay[i] - 1.0) * creep.strain[i] + self.old_weight[i] * stress_old;
        }
        (c1 * self.stress_per_head / self.dt, c0 / self.dt)
    }

    /// A characteristic `H = c -/+ b Q` of a rigid pipe, with the creep at
    /// the point folded in: returns (C', B') for the same form. `settled` is
    /// H0 there, `rise_old` the old head minus H0.
    #[must_use]
    #[inline]
    pub fn fold(&self, c: f64, b: f64, creep: &Creep, settled: f64, rise_old: f64) -> (f64, f64) {
        let (c1, c0) = self.rate(creep, rise_old);
        let scale = 1.0 / (1.0 + self.k * c1);
        ((c + self.k * (c1 * settled - c0)) * scale, b * scale)
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
    fn the_rate_is_what_advancing_produces() {
        let w = wall(3e-10, 0.2);
        let step = CreepStep::new(
            &w,
            WaveSpeed::new(350.0),
            Time::new(0.002),
            STANDARD_GRAVITY,
        );
        let mut creep = Creep::default();
        step.advance(&mut creep, 0.0, 4.0);
        let (c1, c0) = step.rate(&creep, 4.0);
        let before = creep.total();
        step.advance(&mut creep, 4.0, 7.5);
        let rate = (creep.total() - before) / 0.002;
        assert!((rate - (c1 * 7.5 + c0)).abs() <= 1e-12 * rate.abs());
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
        let (c, b) = step.fold(55.0, 1234.0, &Creep::default(), 50.0, 3.0);
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
