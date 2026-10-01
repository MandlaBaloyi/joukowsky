//! Rung 3 - the simulator against an independent method.
//!
//! Every other test of the method of characteristics compares it with our
//! own closed forms or with itself; if it shared a wrong assumption with
//! them, everything would still pass (gap G-01). Here a different method -
//! the frequency-domain transfer-matrix model (`jk_hydraulics::frequency`:
//! no time steps, no grid, exact linear algebra) - must agree with it.

mod transfer_matrix;
