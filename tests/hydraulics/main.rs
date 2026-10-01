//! Hydraulics tests: the physics of transient flow in pipes.
//!
//! Code under test: `crates/domain/hydraulics`. Grouped by validation rung
//! (`docs/07_validation/validation_strategy.txt`), then by topic:
//!
//! - [`analytical`]: results with a known closed-form answer (rung 1).
//! - [`invariants`]: laws that must hold in every simulation - steady state
//!   stays steady, the grid does not change the answer, mass is conserved.
//!
//! - [`cross_code`]: the simulator against an independent method - the
//!   frequency-domain transfer-matrix model (gap G-01); later `TSNet`.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only hydraulics`

pub mod analytical;
mod cross_code;
mod invariants;
