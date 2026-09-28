//! Boundary conditions: what happens where a pipe ends.
//!
//! Each device gives the one extra equation that closes the single
//! characteristic reaching it (`docs/02_physics_of_transients/
//! governing_equations_and_moc.txt` section 2):
//!   - at an UPSTREAM end only the C- characteristic arrives: H = `C_M` + B Q
//!   - at a DOWNSTREAM end only the C+ characteristic arrives: H = `C_P` - B Q
//!
//! Every function here takes the arriving characteristic value and returns
//! the new (head, flow) at the boundary node, as raw f64 in SI units
//! because these sit inside the solver's inner loop.
//!
//! Milestone S2 adds: dead end, junction, leak/burst, pump, check valve,
//! PRV, air valve, surge vessel - one file each.

pub mod closure;
pub mod reservoir;
pub mod valve;
