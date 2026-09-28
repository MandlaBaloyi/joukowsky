//! Hydraulics tests: the physics of transient flow in pipes.
//!
//! Code under test: `crates/domain/hydraulics`. Grouped by validation rung
//! (`docs/07_validation/validation_strategy.txt`), then by topic:
//!
//! - [`analytical`]: results with a known closed-form answer (rung 1).
//! - [`invariants`]: laws that must hold in every simulation - steady state
//!   stays steady, the grid does not change the answer, mass is conserved.
//!
//! Later: `cross_code` (against `TSNet`, rung 3).
//! Run only these:  `.\tests\run-tests.ps1 -Only hydraulics`

mod analytical;
mod invariants;
