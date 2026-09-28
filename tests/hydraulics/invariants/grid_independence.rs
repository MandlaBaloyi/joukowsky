//! The answer must not depend on how finely we slice the pipe.
//!
//! With Courant number 1 and no friction the MOC scheme is exact, so a pipe
//! cut into 1 reach or 64 reaches must give the same Joukowsky peak and
//! trough to rounding error.

use jk_hydraulics::{Pipe, ReservoirPipeValve, ValveClosure, joukowsky};
use jk_kernel::{Head, Length, STANDARD_GRAVITY, Time, Velocity, WaveSpeed};
use jk_tests::{V0, check, check_close, standard_system};

/// Full suite: every grid from 1 to 256 reaches, for the wave speeds of
/// all four pipe-material families in the docs. 1024 simulations; the log
/// shows only the worst case.
#[test]
#[ignore = "full suite: 1024 simulations (run-tests.ps1 -Suite full)"]
fn joukowsky_peak_is_exact_on_every_grid_and_material() {
    let mut worst = 0.0_f64;
    let mut cases = 0;
    for a in [300.0, 370.0, 1000.0, 1250.0] {
        let pipe = Pipe::new(
            Length::new(1000.0),
            Length::new(0.3),
            WaveSpeed::new(a),
            0.0,
        )
        .expect("valid pipe");
        let dh =
            joukowsky::head_change(Velocity::new(-V0), pipe.wave_speed(), STANDARD_GRAVITY).value();
        for reaches in 1..=256 {
            let s = ReservoirPipeValve {
                reservoir_head: Head::new(100.0),
                initial_flow: Velocity::new(V0) * pipe.area(),
                pipe,
                downstream_head: Head::new(50.0),
                closure: ValveClosure::Instantaneous,
                reaches,
                gravity: STANDARD_GRAVITY,
            };
            // Two full periods 4L/a.
            let duration = pipe.round_trip_time() * 4.0;
            let trace = s.simulate(duration, &[s.valve_node()]).unwrap();
            let err_peak = (trace.max_head(0).value() - (100.0 + dh)).abs();
            let err_trough = (trace.min_head(0).value() - (100.0 - dh)).abs();
            worst = worst.max(err_peak).max(err_trough);
            cases += 1;
        }
    }
    println!("  {cases} simulations; largest error in peak or trough: {worst:.2e} m");
    check("every case within 1e-9 m of Joukowsky", worst < 1e-9);
}

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
