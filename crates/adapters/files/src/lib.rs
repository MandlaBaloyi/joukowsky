//! File adapters: getting real data in.
//!
//! An adapter (rule R4): it reads files and builds domain objects; the
//! domain never sees a file. Reading is strict and LOUD
//! (`docs/09_technologies/03_networks_and_reference_tools.txt`): anything
//! a file holds that the engine cannot use, or can only approximate, is
//! reported as a note - never silently dropped (principles P6, P8).
//!
//! - [`epanet`]: network models in the EPANET `.inp` format - the industry
//!   exchange format utilities and consultants already have.
//!
//! - [`logger`]: pressure recordings exported by loggers (CSV and similar
//!   delimited text).
//!
//! - [`site`]: a site file - the network, the loggers (where, which file,
//!   range, clock), ratings, zone, altitude, power losses - read with all
//!   it names into what the live engine needs.
//!
//! - [`scada`]: SCADA and meter exports - flows and tank levels every
//!   few minutes (Tier 0), units named or given, gaps kept.
//!
//! Later: binary logger formats.

pub mod epanet;
mod error;
pub mod logger;
mod note;
pub mod scada;
pub mod site;

pub use error::ReadError;
pub use note::Note;
