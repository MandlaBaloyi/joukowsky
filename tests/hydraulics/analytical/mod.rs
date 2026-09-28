//! Rung 1 - analytical: the engine must reproduce answers physics already
//! gives in closed form.
//!
//! Test names start with the validation ID from
//! `docs/07_validation/validation_strategy.txt` where one exists (va1_,
//! va2_, ...), so a log line traces straight back to the claim it proves.

mod boundaries;
mod joukowsky;
mod network;
mod single_pipe;
mod steady;
