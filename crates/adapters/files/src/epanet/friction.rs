//! EPANET roughness to the engine's Darcy friction factor.
//!
//! The engine's pipes carry one Darcy-Weisbach friction factor f (quasi-
//! steady friction, `docs/02_physics_of_transients/friction_and_damping.txt`).
//! An `.inp` file gives roughness for one of three formulas
//! (`[OPTIONS] HEADLOSS`); each is turned into the f that gives the SAME
//! head loss at a reference velocity V (1 m/s by default):
//!
//! ```text
//!   f = (h / L) 2 g D / V^2,  with the file's formula giving h / L at
//!   Q = V pi D^2 / 4 (EPANET 2.2 manual, "Hydraulic Model", SI form):
//!     Hazen-Williams   h / L = 10.667 C^-1.852 D^-4.871 Q^1.852
//!     Chezy-Manning    h / L = 10.294 n^2 D^-5.33 Q^2
//!     Darcy-Weisbach   f from roughness e by Swamee-Jain:
//!                      f = 0.25 / log10(e / (3.7 D) + 5.74 / Re^0.9)^2
//! ```
//!
//! A pipe's real f changes with the flow; one value is an approximation
//! the reader reports [VERIFY: calibrate on field data].

/// Kinematic viscosity of water at 20 C [m^2/s].
const WATER_VISCOSITY: f64 = 1.004e-6;

/// Which head-loss formula a file uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Headloss {
    /// Hazen-Williams: roughness is C (EPANET's default).
    HazenWilliams,
    /// Darcy-Weisbach: roughness is a length (converted to metres).
    DarcyWeisbach,
    /// Chezy-Manning: roughness is n.
    ChezyManning,
}

impl Headloss {
    /// Reads the name as written in files (any case).
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s.to_ascii_uppercase().as_str() {
            "H-W" => Self::HazenWilliams,
            "D-W" => Self::DarcyWeisbach,
            "C-M" => Self::ChezyManning,
            _ => return None,
        })
    }

    /// The formula's name.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::HazenWilliams => "Hazen-Williams",
            Self::DarcyWeisbach => "Darcy-Weisbach",
            Self::ChezyManning => "Chezy-Manning",
        }
    }
}

/// The Darcy friction factor equivalent to `roughness` (C, metres, or n)
/// for a pipe of `diameter` metres at `velocity` m/s; `viscosity` is
/// relative to water at 20 C; `g` the gravity used.
#[must_use]
pub fn darcy_factor(
    formula: Headloss,
    roughness: f64,
    diameter: f64,
    velocity: f64,
    viscosity: f64,
    g: f64,
) -> f64 {
    let area = std::f64::consts::PI * diameter * diameter / 4.0;
    let q = velocity * area;
    let from_gradient = |gradient: f64| gradient * 2.0 * g * diameter / (velocity * velocity);
    match formula {
        Headloss::HazenWilliams => {
            from_gradient(10.667 * roughness.powf(-1.852) * diameter.powf(-4.871) * q.powf(1.852))
        }
        Headloss::ChezyManning => {
            from_gradient(10.294 * roughness * roughness * diameter.powf(-5.33) * q * q)
        }
        Headloss::DarcyWeisbach => {
            let re = velocity * diameter / (WATER_VISCOSITY * viscosity);
            let x = roughness / (3.7 * diameter) + 5.74 / re.powf(0.9);
            0.25 / (x.log10() * x.log10())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f64 = 9.81;

    #[test]
    fn darcy_weisbach_matches_the_moody_chart() {
        // Commercial steel-ish pipe: e = 0.1 mm, D = 150 mm, V = 1 m/s:
        // Re = 1.49e5, e/D = 6.7e-4 -> f about 0.021 (Moody).
        let f = darcy_factor(Headloss::DarcyWeisbach, 1e-4, 0.15, 1.0, 1.0, G);
        assert!((f - 0.021).abs() < 0.001, "{f}");
    }

    #[test]
    fn hazen_williams_gives_the_same_head_loss_at_the_reference_velocity() {
        let (c, d, v) = (130.0, 0.3, 1.0);
        let f = darcy_factor(Headloss::HazenWilliams, c, d, v, 1.0, G);
        let q = v * std::f64::consts::PI * d * d / 4.0;
        let hw = 10.667 * f64::powf(c, -1.852) * d.powf(-4.871) * q.powf(1.852);
        let darcy = f * v * v / (2.0 * G * d);
        assert!((hw - darcy).abs() < 1e-12);
        // C = 130 in a 300 mm main is a smooth pipe: f about 0.018.
        assert!((0.014..0.022).contains(&f), "{f}");
    }

    #[test]
    fn manning_gives_the_same_head_loss_at_the_reference_velocity() {
        let (n, d, v) = (0.011, 0.2, 1.0);
        let f = darcy_factor(Headloss::ChezyManning, n, d, v, 1.0, G);
        let q = v * std::f64::consts::PI * d * d / 4.0;
        let m = 10.294 * n * n * d.powf(-5.33) * q * q;
        assert!((m - f * v * v / (2.0 * G * d)).abs() < 1e-12);
    }
}
