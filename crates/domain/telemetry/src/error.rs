use std::fmt;

use jk_kernel::Time;

/// Why a chunk of data was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryError {
    /// The sampling interval is not finite and positive.
    InvalidInterval(Time),
    /// A chunk's sampling interval differs from the stream's.
    IntervalChanged {
        /// The stream's interval.
        expected: Time,
        /// The chunk's interval.
        got: Time,
    },
    /// A chunk starts before data already received ended: a duplicate
    /// upload or a clock that jumped back (P6: refused, not merged).
    Overlap {
        /// When the next sample was due.
        expected: Time,
        /// When the chunk says it starts.
        got: Time,
    },
}

impl fmt::Display for TelemetryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInterval(dt) => {
                write!(
                    f,
                    "sampling interval {} s must be finite and > 0",
                    dt.value()
                )
            }
            Self::IntervalChanged { expected, got } => write!(
                f,
                "sampling interval changed from {} s to {} s",
                expected.value(),
                got.value()
            ),
            Self::Overlap { expected, got } => write!(
                f,
                "chunk starts at {} s but data up to {} s was already received: a duplicate or \
                 a clock that jumped back",
                got.value(),
                expected.value()
            ),
        }
    }
}

impl std::error::Error for TelemetryError {}
