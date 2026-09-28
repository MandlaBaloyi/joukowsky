//! 03 - Single pipe: reservoir -> pipe -> valve, solved by MOC.
//!
//! Validation cases V-A1 and V-A2 of `docs/07_validation/validation_strategy.txt`;
//! code in `crates/domain/hydraulics/src/moc/single_pipe.rs`.
//! Every test uses the standard system from `common/lib.rs`: 1000 m of
//! DN300, a = 1000 m/s, 1 m/s, so 2L/a = 2 s and the period 4L/a = 4 s.
//! Run only this folder:  `cargo test -p jk-tests --test single_pipe`

mod friction;
mod grid_independence;
mod slow_closure;
mod square_wave;
mod steady_state;
