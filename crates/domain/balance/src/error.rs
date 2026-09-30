use std::fmt;

/// Why a balance could not be computed.
#[derive(Debug, Clone, PartialEq)]
pub enum BalanceError {
    /// Input that breaks a stated rule (lengths, intervals, values).
    InvalidInput(String),
}

impl fmt::Display for BalanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(why) => write!(f, "invalid input: {why}"),
        }
    }
}

impl std::error::Error for BalanceError {}
