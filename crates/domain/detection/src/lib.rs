//! Detection bounded context: finding transient events in pressure signals.
//!
//! Implements `docs/04_detection_and_localisation/event_detection.txt`:
//! decide, online and with bounded delay, that a transient has started, and
//! estimate WHEN it arrived precisely enough to locate its source
//! (location error = a * timing error / 2).
//!
//! Pure domain code (rule R2): samples are already in memory; times are
//! inputs. Continuity and gaps are the telemetry context's: a detector
//! stream covers one continuous stretch. The per-sample path
//! ([`change::Cusum::push`]) allocates nothing, so it can later run next to
//! the sensor on a partner's gateway (P6).
//!
//! The pieces, one folder per layer of the docs' layered detector (a
//! folder for sensor health - Layer 0 - appears when its work starts):
//! - [`noise`]: how much a quiet sensor wiggles (robust, from differences).
//! - [`baseline`]: the drifting normal level (Brown's double smoothing),
//!   Layer 1.
//! - [`change`]: two-sided CUSUM change detection (Page 1954), in units
//!   of the noise (Layer 2).
//! - [`onset`]: the arrival time of a front, to a fraction of a sample
//!   (Layer 3).
//! - [`detector`]: the whole chain - live ([`DetectorStream`], pieces of
//!   any size) or over a whole record ([`Detector::run`]), identically.
//!
//! Network-level fusion across sensors (Layer 4) belongs to localisation.

pub mod baseline;
pub mod change;
pub mod detector;
mod error;
pub mod noise;
pub mod onset;

pub use detector::{
    Detection, Detector, DetectorConfig, DetectorStream, Direction, Event, Impairment, Series,
    Span, StreamOutput,
};
pub use error::DetectionError;
