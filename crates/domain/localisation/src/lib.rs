//! Localisation bounded context: WHERE on the network a transient came from.
//!
//! Implements `docs/04_detection_and_localisation/burst_localisation.txt`
//! section 2. The answer is a probability map over the pipes, never a single
//! dot (principle P3): a ranked list of pipes, the most likely point, and the
//! smallest set of pipe segments holding 90% of the probability.
//!
//! Method, in one paragraph: from each sensor, the fastest wave travel time
//! to every node (Dijkstra, weights L/a); for candidate points every few
//! metres along every pipe, the travel time to each sensor; the unknown
//! emission time cancels analytically; each candidate is scored by how well
//! it explains the observed arrival times (Gaussian errors); uncertain wave
//! speeds are carried through each wave's route as correlated timing errors
//! (generalised least squares - sensors whose waves shared a pipe share its
//! error).
//!
//! Pure domain code (rule R2). The folders:
//! - [`graph`]: the wave-travel graph and fastest travel times.
//! - [`posterior`]: candidate grid, scores, credible region.
//! - [`locate`]: the entry point, [`localise`].
//! - [`clocks`]: sensor clock offsets, estimated from known-source events
//!   and removed (software clock repair, frontier F9).
//! - [`placement`]: where loggers should go - the placement advisor.

pub mod clocks;
mod error;
pub mod graph;
pub mod locate;
pub mod placement;
pub mod posterior;

pub use clocks::{CalibrationEvent, ClockEstimate, estimate_offsets};
pub use error::LocalisationError;
pub use locate::{Arrival, Localisation, LocaliseOptions, Localiser, Point, localise};
pub use posterior::{PipeProbability, Segment};
