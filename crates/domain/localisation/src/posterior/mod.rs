//! The probability map: candidates, scores, credible region.
//!
//! `burst_localisation.txt` section 2. For a candidate source x, sensor s
//! should see the wave at t0 + `T_s`(x) (t0 = unknown emission time). With
//! Gaussian timing errors of standard deviation `sigma_s`, the best t0 is
//! the weighted mean of (`t_s` - `T_s`(x)), and the score is
//!
//! ```text
//!   log L(x) = -1/2 sum_s ( t_s - t0_hat - T_s(x) )^2 / sigma_s^2
//! ```
//!
//! With a uniform prior per metre of pipe, each candidate cell's posterior
//! weight is L(x) x (cell length). (The prior is where pipe risk - age,
//! material, history - will enter later.)

pub(crate) mod bound;
pub mod credible;
pub mod grid;
pub mod score;

pub use credible::{PipeProbability, Segment};
pub use grid::{Cell, Grid};
