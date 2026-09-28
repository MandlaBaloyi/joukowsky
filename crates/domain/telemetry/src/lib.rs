//! Telemetry bounded context: what arrives from the sensors, and what does
//! not.
//!
//! Loggers deliver their samples in pieces ("chunks": a second, a minute,
//! an upload every five minutes). Between pieces things go wrong in the
//! field: load-shedding switches a gateway off, an upload is lost, a
//! transducer reports garbage. Principle P6: expect broken data, never
//! silently fill a gap, and silence is not "no events". So this context
//! turns chunks into CONTINUOUS stretches of valid samples plus explicit
//! GAPS, and says how far each sensor's data has arrived.
//!
//! Pure domain code (rule R2): chunks are already in memory; the times on
//! them are inputs, never read from a clock.
//!
//! - [`assembly`]: chunks in, continuous stretches and gaps out.
//!
//! Later: data tiers (which capabilities a stream supports - Tier 2 / 1 /
//! 0), clock quality, sensor health.

pub mod assembly;
mod error;

pub use assembly::{Assembler, Chunk, Gap, GapCause, Piece};
pub use error::TelemetryError;
