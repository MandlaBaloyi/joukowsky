//! Boundary conditions: what happens where a pipe ends or pipes meet.
//!
//! Each device gives the extra equation that closes the characteristics
//! reaching it (`docs/02_physics_of_transients/
//! governing_equations_and_moc.txt` section 2):
//!   - at an UPSTREAM end only the C- characteristic arrives: H = `C_M` + B Q
//!   - at a DOWNSTREAM end only the C+ characteristic arrives: H = `C_P` - B Q
//!   - at a junction every connected pipe brings one of the two
//!
//! Every function here takes the arriving characteristic value(s) and
//! returns the new (head, flow) at the node, as raw f64 in SI units because
//! these sit inside the solver's inner loop.
//!
//! Milestone S2. Done: reservoir, dead end, valve (+ closure curves),
//! junction, leak/burst, ideal check valve, pump with non-return valve
//! (normal zone, run-down after a trip). Next: four-quadrant pump curves,
//! dynamic valve slam, PRV, air valve, surge vessel.

pub mod check_valve;
pub mod closure;
pub mod dead_end;
pub mod junction;
pub mod leak;
pub mod pump;
pub mod reservoir;
pub mod valve;
