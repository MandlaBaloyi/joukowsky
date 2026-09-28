use jk_kernel::Time;

use crate::HydraulicsError;
use crate::pipe::positive;

/// How a valve's relative opening tau (1 = as at steady state, 0 = shut)
/// changes with time. Closure starts at t = 0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValveClosure {
    /// The valve never moves: the steady state must persist.
    Open,
    /// Shut at the first time step after t = 0 (the Joukowsky case).
    Instantaneous,
    /// tau(t) = (1 - t / duration)^exponent, then 0.
    /// Exponent 1 is a linear closure. Gate valves, which lose most of
    /// their flow in the last turns, are better described by exponents
    /// well below 1 (`docs/01_problem_definition/the_operating_environment.txt`).
    Power {
        /// Time to fully shut.
        duration: Time,
        /// Shape of the closure curve.
        exponent: f64,
    },
}

impl ValveClosure {
    /// A linear closure over `duration`.
    #[must_use]
    pub fn linear(duration: Time) -> Self {
        Self::Power {
            duration,
            exponent: 1.0,
        }
    }

    /// Checks the closure parameters.
    ///
    /// # Errors
    /// [`HydraulicsError::InvalidParameter`] if a power closure has a
    /// non-positive duration or exponent.
    pub fn validate(&self) -> Result<(), HydraulicsError> {
        match *self {
            Self::Open | Self::Instantaneous => Ok(()),
            Self::Power { duration, exponent } => {
                positive("closure duration", duration.value())?;
                positive("closure exponent", exponent)
            }
        }
    }

    /// Relative opening tau at time `t`.
    #[must_use]
    pub fn opening(&self, t: Time) -> f64 {
        let t = t.value();
        match *self {
            Self::Open => 1.0,
            Self::Instantaneous => {
                if t > 0.0 {
                    0.0
                } else {
                    1.0
                }
            }
            Self::Power { duration, exponent } => {
                if t <= 0.0 {
                    1.0
                } else if t >= duration.value() {
                    0.0
                } else {
                    (1.0 - t / duration.value()).powf(exponent)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn openings() {
        assert!((ValveClosure::Open.opening(Time::new(99.0)) - 1.0).abs() < f64::EPSILON);
        assert!((ValveClosure::Instantaneous.opening(Time::ZERO) - 1.0).abs() < f64::EPSILON);
        assert!(ValveClosure::Instantaneous.opening(Time::new(1e-9)).abs() < f64::EPSILON);
        let lin = ValveClosure::linear(Time::new(4.0));
        assert!((lin.opening(Time::new(1.0)) - 0.75).abs() < 1e-15);
        assert!(lin.opening(Time::new(4.0)).abs() < f64::EPSILON);
        assert!(lin.opening(Time::new(5.0)).abs() < f64::EPSILON);
    }

    #[test]
    fn rejects_bad_power_closures() {
        assert!(ValveClosure::linear(Time::ZERO).validate().is_err());
        let bad = ValveClosure::Power {
            duration: Time::new(1.0),
            exponent: 0.0,
        };
        assert!(bad.validate().is_err());
    }
}
