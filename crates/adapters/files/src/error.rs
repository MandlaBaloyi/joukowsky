use std::fmt;

/// Why a file could not be read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadError {
    /// The line it concerns (1-based), if one.
    pub line: Option<usize>,
    /// What is wrong, in plain words.
    pub message: String,
}

impl ReadError {
    pub(crate) fn at(line: usize, message: impl Into<String>) -> Self {
        Self {
            line: Some(line),
            message: message.into(),
        }
    }

    pub(crate) fn general(message: impl Into<String>) -> Self {
        Self {
            line: None,
            message: message.into(),
        }
    }
}

impl fmt::Display for ReadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.line {
            Some(l) => write!(f, "line {l}: {}", self.message),
            None => f.write_str(&self.message),
        }
    }
}

impl std::error::Error for ReadError {}
