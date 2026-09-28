use std::fmt;

/// Why a localisation could not be made.
#[derive(Debug, Clone, PartialEq)]
pub enum LocalisationError {
    /// A setting or input is outside its meaningful range.
    InvalidParameter {
        /// Parameter name.
        name: &'static str,
        /// The rejected value.
        value: f64,
        /// What it must satisfy.
        requirement: &'static str,
    },
    /// The inputs do not fit the network (unknown node, repeated sensor,
    /// too few sensors).
    InvalidInput(String),
    /// A sensor cannot be reached by any wave path - the network is split.
    Unreachable {
        /// Index of the sensor's node.
        node: usize,
    },
}

impl LocalisationError {
    pub(crate) fn invalid(name: &'static str, value: f64, requirement: &'static str) -> Self {
        Self::InvalidParameter {
            name,
            value,
            requirement,
        }
    }
}

impl fmt::Display for LocalisationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidParameter {
                name,
                value,
                requirement,
            } => write!(f, "invalid {name} = {value}: must be {requirement}"),
            Self::InvalidInput(why) => write!(f, "invalid input: {why}"),
            Self::Unreachable { node } => {
                write!(
                    f,
                    "no wave path connects the sensor at node {node} to the network"
                )
            }
        }
    }
}

impl std::error::Error for LocalisationError {}

/// Accepts only finite, strictly positive values.
pub(crate) fn positive(name: &'static str, value: f64) -> Result<(), LocalisationError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(LocalisationError::invalid(name, value, "finite and > 0"))
    }
}
