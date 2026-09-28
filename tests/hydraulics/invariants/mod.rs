//! Invariants: laws that must hold in every simulation, whatever the
//! network or event.
//!
//! These are the tests that will grow into property tests - thousands of
//! randomly generated networks, each checked for conservation, symmetry and
//! stability. The large versions are marked slow and run in the full suite
//! (`.\tests\run-tests.ps1 -Suite full`).

mod grid_independence;
mod network;
mod steady_state;
