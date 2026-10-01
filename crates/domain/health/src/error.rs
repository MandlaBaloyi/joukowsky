use std::fmt;

/// Why a health measure could not be computed.
#[derive(Debug, Clone, PartialEq)]
pub enum HealthError {
    /// Input that breaks a stated rule (a setting out of range, a
    /// non-finite sample).
    InvalidInput(String),
}

impl fmt::Display for HealthError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(why) => write!(f, "invalid input: {why}"),
        }
    }
}

impl std::error::Error for HealthError {}
