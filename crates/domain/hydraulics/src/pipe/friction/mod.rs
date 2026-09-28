//! Wall friction models.
//!
//! Implements `docs/02_physics_of_transients/friction_and_damping.txt`.
//! Planned order: quasi-steady (reference, done) -> Brunone/IAB unsteady
//! friction -> convolution (Vardy-Brown) friction (milestone S5).

pub mod quasi_steady;
