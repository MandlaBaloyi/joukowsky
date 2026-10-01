//! Laws the network solver must obey on any network.
//!
//! The strongest of these compare the network solver with the single-pipe
//! solver, which is already proven against Joukowsky: two independent
//! implementations must agree.

mod cavitation;
mod continuity;
mod matches_single_pipe;
mod steady_start;
mod symmetry;

use jk_hydraulics::{GridOptions, ReservoirPipeValve, ValveClosure};
use jk_kernel::Time;

/// The standard single-pipe system's valve-head history, 20 reaches.
pub fn reference_valve_heads(friction: f64, closure: ValveClosure, duration: f64) -> Vec<f64> {
    let s: ReservoirPipeValve = jk_tests::standard_system(friction, closure, 20);
    s.simulate(Time::new(duration), &[s.valve_node()])
        .expect("standard system runs")
        .head(0)
        .iter()
        .map(|h| h.value())
        .collect()
}

/// Grid options giving dt = 0.05 s when the shortest pipe is `shortest` m
/// long at a = 1000 m/s - the same dt as the 20-reach standard system.
pub fn options_for_dt_005(shortest: f64) -> GridOptions {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small positive
    let min_reaches = (shortest / 50.0).round() as u32;
    GridOptions {
        max_wave_speed_adjustment: 1e-9,
        min_reaches,
    }
}
