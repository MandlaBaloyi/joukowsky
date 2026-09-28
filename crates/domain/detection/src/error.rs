use std::fmt;

/// Why detection could not run on a series.
#[derive(Debug, Clone, PartialEq)]
pub enum DetectionError {
    /// A setting is outside its meaningful range.
    InvalidParameter {
        /// Parameter name.
        name: &'static str,
        /// The rejected value.
        value: f64,
        /// What it must satisfy.
        requirement: &'static str,
    },
    /// Too few samples to learn the sensor's noise and level.
    TooShort {
        /// Samples given.
        samples: usize,
        /// Samples needed.
        needed: usize,
    },
    /// A sample is NaN or infinite. Gaps must be reported by the telemetry
    /// context, never silently filled (principle P6).
    NonFiniteSample {
        /// Index of the bad sample.
        index: usize,
    },
}

impl DetectionError {
    pub(crate) fn invalid(name: &'static str, value: f64, requirement: &'static str) -> Self {
        Self::InvalidParameter {
            name,
            value,
            requirement,
        }
    }
}

impl fmt::Display for DetectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter {
                name,
                value,
                requirement,
            } => write!(f, "invalid {name} = {value}: must be {requirement}"),
            Self::TooShort { samples, needed } => write!(
                f,
                "{samples} samples is too short: {needed} are needed to learn the sensor's noise"
            ),
            Self::NonFiniteSample { index } => write!(
                f,
                "sample {index} is not a number; gaps must be reported, not filled"
            ),
        }
    }
}

impl std::error::Error for DetectionError {}

/// Accepts only finite, strictly positive values.
pub(crate) fn positive(name: &'static str, value: f64) -> Result<(), DetectionError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(DetectionError::invalid(name, value, "finite and > 0"))
    }
}
