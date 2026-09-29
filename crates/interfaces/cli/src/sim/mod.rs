//! Simulated sites and loggers - demo material for `jk view`.
//!
//! Until real network files and logger exports can be read (the data
//! readers, next), the viewer shows the engine on the same simulated sites
//! the validation tests use.
//!
//! - [`logger`]: a synthetic 128 S/s logger.
//! - [`sites`]: the test estate and the commercial campus, with a position
//!   for every node so they can be drawn.

pub mod logger;
pub mod sites;
