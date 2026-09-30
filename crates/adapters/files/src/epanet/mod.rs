//! Network models in the EPANET `.inp` format.
//!
//! EPANET (US EPA, public domain) is how utilities and consultants
//! describe water networks; many real networks are published in it. This
//! reader turns an `.inp` file into the engine's [`Network`], plus what
//! the engine needs to show it (IDs, elevations, map coordinates) - and a
//! list of NOTES: everything in the file it could only approximate or did
//! not use. The notes are part of the result, to be shown to the reader
//! of any output built on it (principle P8).
//!
//! What maps directly: junctions (with their base demand), reservoirs,
//! pipes (length, diameter), map coordinates and pipe bends. What is
//! approximated, always with a note:
//! - wave speeds: `.inp` files have none; one value is assumed for every
//!   pipe (1000 m/s by default) until a sidecar file supplies real ones;
//! - friction: the file's roughness becomes one Darcy factor per pipe
//!   ([`friction`]);
//! - tanks: fixed-head nodes at their initial level in transients; with
//!   their minimum/maximum levels and diameter given, also storage tanks
//!   whose level extended-period runs move (volume curves not read);
//! - pumps and open valves: their two nodes are joined into one - the
//!   network stays connected for locating bursts, but the device's own
//!   hydraulics are not modelled;
//! - closed pipes, pumps and valves: left out (the two sides are not
//!   connected); check-valve pipes: kept as plain pipes.
//!
//! What is not used (patterns, curves, controls, rules, energy, quality,
//! ...) is listed section by section. Anything not in the EPANET format
//! is an error, never skipped.
//!
//! - [`sections`]: splitting the text into sections and records.
//! - [`units`]: US customary and SI units to SI.
//! - [`friction`]: roughness to a Darcy friction factor.

mod build;
pub mod friction;
pub mod sections;
pub mod units;

use std::path::Path;

use jk_hydraulics::Network;
use jk_kernel::{Acceleration, Length, STANDARD_GRAVITY, Velocity, WaveSpeed};

use crate::error::ReadError;
pub use crate::note::Note;
pub use friction::Headloss;
pub use units::FlowUnits;

/// Choices the file cannot make.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InpOptions {
    /// Wave speed given to every pipe (`.inp` files carry none).
    pub wave_speed: WaveSpeed,
    /// Velocity at which roughness is turned into a Darcy factor.
    pub reference_velocity: Velocity,
    /// Gravity for the network.
    pub gravity: Acceleration,
}

impl Default for InpOptions {
    fn default() -> Self {
        Self {
            wave_speed: WaveSpeed::new(1000.0),
            reference_velocity: Velocity::new(1.0),
            gravity: STANDARD_GRAVITY,
        }
    }
}

/// A network read from an `.inp` file.
#[derive(Debug, Clone)]
pub struct InpNetwork {
    /// The file's [TITLE].
    pub title: String,
    /// The file's flow units (which also fix its other units).
    pub units: FlowUnits,
    /// The file's head-loss formula.
    pub headloss: Headloss,
    /// The network, in SI units.
    pub network: Network,
    /// The file's ID of each node (joined IDs as "A=B" where a pump or
    /// valve was replaced by a connection).
    pub node_ids: Vec<String>,
    /// The file's ID of each pipe.
    pub pipe_ids: Vec<String>,
    /// Ground elevation of each node, where the file gives one.
    pub elevations: Vec<Option<Length>>,
    /// Map position of each node, in the file's own plan units.
    pub positions: Vec<Option<(f64, f64)>>,
    /// Bends of each pipe between its ends, in the same plan units.
    pub vertices: Vec<Vec<(f64, f64)>>,
    /// Everything approximated or not used.
    pub notes: Vec<Note>,
}

/// Reads an `.inp` text.
///
/// # Errors
/// Anything that is not valid EPANET: an unknown section, a record with a
/// missing or non-numeric field, an unknown node, a repeated ID, a pipe
/// that is not physical, or no pipes at all.
pub fn parse(text: &str, options: &InpOptions) -> Result<InpNetwork, ReadError> {
    build::build(text, options)
}

/// Reads an `.inp` file.
///
/// # Errors
/// A file that cannot be read, or the errors of [`parse`].
pub fn read(path: &Path, options: &InpOptions) -> Result<InpNetwork, ReadError> {
    let bytes = std::fs::read(path)
        .map_err(|e| ReadError::general(format!("cannot read {}: {e}", path.display())))?;
    // Files from Windows tools are often not UTF-8 (IDs are plain ASCII;
    // comments may not be): replace what cannot be decoded.
    let text = String::from_utf8_lossy(&bytes);
    parse(&text, options)
}
