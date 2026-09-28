use std::fmt;

use jk_classification::ClassificationError;
use jk_detection::DetectionError;
use jk_localisation::LocalisationError;
use jk_telemetry::TelemetryError;

/// Why the pipeline could not process a window of records.
#[derive(Debug, Clone, PartialEq)]
pub enum ApplicationError {
    /// The site description or the records do not fit together.
    InvalidInput(String),
    /// Detection failed on a sensor's record.
    Detection {
        /// The sensor's name.
        sensor: String,
        /// Why.
        error: DetectionError,
    },
    /// A sensor's data was refused (overlapping chunks, a changed rate).
    Telemetry {
        /// The sensor's name.
        sensor: String,
        /// Why.
        error: TelemetryError,
    },
    /// Localisation failed.
    Localisation(LocalisationError),
    /// Classification failed.
    Classification(ClassificationError),
    /// An alert could not be delivered.
    Delivery(String),
}

impl fmt::Display for ApplicationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(why) => write!(f, "invalid input: {why}"),
            Self::Detection { sensor, error } => write!(f, "detection at {sensor}: {error}"),
            Self::Telemetry { sensor, error } => write!(f, "data from {sensor}: {error}"),
            Self::Localisation(e) => write!(f, "localisation: {e}"),
            Self::Classification(e) => write!(f, "classification: {e}"),
            Self::Delivery(why) => write!(f, "alert delivery: {why}"),
        }
    }
}

impl std::error::Error for ApplicationError {}

impl From<LocalisationError> for ApplicationError {
    fn from(e: LocalisationError) -> Self {
        Self::Localisation(e)
    }
}

impl From<ClassificationError> for ApplicationError {
    fn from(e: ClassificationError) -> Self {
        Self::Classification(e)
    }
}
