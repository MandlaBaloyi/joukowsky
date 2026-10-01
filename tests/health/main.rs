//! Health tests: pressure records in, fatigue loading out.
//!
//! Code under test: `crates/domain/health` (rainflow counting, Miner's
//! rule, equivalent cycles) - `docs/02_physics_of_transients/
//! pressure_leakage_and_fatigue.txt` section 2.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only health`

mod analytical;
