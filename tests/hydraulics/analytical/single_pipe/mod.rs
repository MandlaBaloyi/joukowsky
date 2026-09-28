//! Reservoir -> pipe -> valve, solved by MOC.
//!
//! Code in `crates/domain/hydraulics/src/moc/single_pipe.rs`. Every test
//! uses the standard system from `tests/common/lib.rs`: 1000 m of DN300,
//! a = 1000 m/s, 1 m/s, so 2L/a = 2 s and the period 4L/a = 4 s.

mod friction;
mod slow_closure;
mod square_wave;
