//! Channels bounded context: water in canals and rivers - open-channel
//! flow, the 1-D Saint-Venant equations (step R-4 of
//! `docs/02_physics_of_transients/sources_dams_rivers_canals.txt`; method
//! in `open_channels.txt`).
//!
//! Pure: a reach, its ends and a start in; depths and flows over time out.
//!
//! - [`section`]: cross-sections and reaches; normal and critical depth.
//! - [`solver`]: the finite-volume solver (HLL flux, explicit steps).

pub mod section;
pub mod solver;

mod error;

pub use error::ChannelError;
pub use section::{Reach, Section};
pub use solver::{
    ChannelOptions, ChannelRun, ChannelState, Downstream, Initial, Upstream, centres, simulate,
};
