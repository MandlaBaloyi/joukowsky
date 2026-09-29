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
//! Later: logger exports (CSV), SCADA extracts.

pub mod epanet;
mod error;

pub use error::ReadError;
