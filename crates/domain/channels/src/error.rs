use std::fmt;

/// Why a channel could not be set up or run.
#[derive(Debug, Clone, PartialEq)]
pub enum ChannelError {
    /// Input that breaks a stated rule (a dimension, a slope, a time).
    InvalidInput(String),
}

impl fmt::Display for ChannelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(why) => write!(f, "invalid input: {why}"),
        }
    }
}

impl std::error::Error for ChannelError {}

/// `value` if positive and finite, else an error naming it.
pub(crate) fn positive(what: &str, value: f64) -> Result<f64, ChannelError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(ChannelError::InvalidInput(format!(
            "{what} must be positive and finite, got {value}"
        )))
    }
}
