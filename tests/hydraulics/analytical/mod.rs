//! Rung 1 - analytical: the engine must reproduce answers physics already
//! gives in closed form.
//!
//! Test names start with the validation ID from
//! `docs/07_validation/validation_strategy.txt` where one exists (va1_,
//! va2_, ...), so a log line traces straight back to the claim it proves.

mod boundaries;
mod cavitation;
mod control_valves;
mod controls;
mod joukowsky;
mod network;
mod plastic_front;
mod pumps;
mod refill;
mod single_pipe;
mod sources;
pub mod steady;
mod tanks;
mod viscoelastic;
