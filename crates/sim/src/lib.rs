//! Synthetic networks and scenarios, for validation and demos.
//!
//! Test networks we generate ourselves, deterministically from a seed, so
//! the engine can be measured on any size without downloading anything.
//! Used by the validation tests and the `jk` command only - never by the
//! domain or the application (it is a test bench, not part of the engine).
//!
//! - [`town`]: a synthetic town of any length - street grid, mains, loops,
//!   cul-de-sacs, mixed pipe materials, reservoirs.

pub mod town;

pub use town::{Town, TownOptions, town};
