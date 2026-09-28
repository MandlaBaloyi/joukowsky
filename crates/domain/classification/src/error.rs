use std::fmt;

/// Why an event could not be classified.
#[derive(Debug, Clone, PartialEq)]
pub enum ClassificationError {
    /// A setting is outside its meaningful range.
    InvalidParameter {
        /// Parameter name.
        name: &'static str,
        /// The rejected value.
        value: f64,
        /// What it must satisfy.
        requirement: &'static str,
    },
    /// The record around the event is too short to measure its features.
    NotEnoughRecord {
        /// Seconds of record needed before the arrival.
        before: f64,
        /// Seconds of record needed after the arrival.
        after: f64,
    },
}

impl ClassificationError {
    pub(crate) fn invalid(name: &'static str, value: f64, requirement: &'static str) -> Self {
        Self::InvalidParameter {
            name,
            value,
            requirement,
        }
    }
}

impl fmt::Display for ClassificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter {
                name,
                value,
                requirement,
            } => write!(f, "invalid {name} = {value}: must be {requirement}"),
            Self::NotEnoughRecord { before, after } => write!(
                f,
                "need {before} s of record before and {after} s after the arrival"
            ),
        }
    }
}

impl std::error::Error for ClassificationError {}

/// Accepts only finite, strictly positive values.
pub(crate) fn positive(name: &'static str, value: f64) -> Result<(), ClassificationError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(ClassificationError::invalid(name, value, "finite and > 0"))
    }
}
