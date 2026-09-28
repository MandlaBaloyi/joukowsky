//! The answer must not depend on how finely we slice the pipe.
//!
//! With Courant number 1 and no friction the MOC scheme is exact, so a pipe
//! cut into 1 reach or 64 reaches must give the same Joukowsky peak and
//! trough to rounding error.

use jk_hydraulics::{ValveClosure, joukowsky};
use jk_kernel::{Time, Velocity};
use jk_tests::{V0, check_close, standard_system};

#[test]
fn joukowsky_peak_is_the_same_on_every_grid() {
    for reaches in [1, 7, 20, 64] {
        let s = standard_system(0.0, ValveClosure::Instantaneous, reaches);
        let dh = joukowsky::head_change(Velocity::new(-V0), s.pipe.wave_speed(), s.gravity).value();
        let trace = s.simulate(Time::new(8.0), &[s.valve_node()]).unwrap();
        println!(
            "{reaches} reach(es), dt = {:.4} s:",
            trace.time_step().value()
        );
        check_close(
            "peak head [m]",
            trace.max_head(0).value(),
            100.0 + dh,
            1e-12,
        );
        check_close(
            "lowest head [m]",
            trace.min_head(0).value(),
            100.0 - dh,
            1e-12,
        );
    }
}
