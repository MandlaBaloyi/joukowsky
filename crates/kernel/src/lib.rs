//! Shared kernel of the joukowsky engine.
//!
//! Everything here is used by every bounded context, so it is kept small
//! and depends on nothing but the standard library (dependency rule R1 in
//! `docs/06_engine_architecture/project_structure_and_ddd.txt`).
//!
//! The core of the kernel is [`units`]: SI quantities as distinct types, so
//! that adding a head [m] to a flow [m^3/s] is a compile error rather than
//! a silent physics bug.
//!
//! [`random`] holds the deterministic random-number generator every
//! stochastic method uses, so results replay bit for bit (rule T3), and
//! [`linalg`] the small dense solver (Cholesky) several contexts share,
//! and [`sparse`] the envelope Cholesky for large sparse systems (the
//! steady solver's nodal matrix). [`clock`]: dates from seconds since 1970,
//! and how a moment is written for people.

pub mod clock;
pub mod linalg;
pub mod random;
pub mod sparse;
pub mod units;

pub use units::{
    Acceleration, Area, Density, Flow, Head, Length, Pressure, STANDARD_GRAVITY, Time, Velocity,
    WaveSpeed,
};
