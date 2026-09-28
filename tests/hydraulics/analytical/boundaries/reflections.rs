//! V-A3: transmission and reflection at fittings.
//!
//! A wave of height F arrives. The head at the fitting becomes H0 + T F,
//! where T is the transmission coefficient; the reflected part is R = T - 1.
//! Each case is measured from the boundary code and compared with the
//! independent closed-form formula in `joukowsky::junction_transmission`.

use jk_hydraulics::boundary::junction::{self, Arrival, PipeEnd};
use jk_hydraulics::boundary::{dead_end, reservoir};
use jk_hydraulics::{Pipe, joukowsky};
use jk_kernel::{Length, STANDARD_GRAVITY, WaveSpeed};
use jk_tests::check_close;

use super::{F, H0, incoming_c_plus};

/// DN150 pipe impedance for a given wave speed.
fn impedance(a: f64) -> f64 {
    Pipe::new(
        Length::new(100.0),
        Length::new(0.15),
        WaveSpeed::new(a),
        0.0,
    )
    .expect("valid pipe")
    .impedance(STANDARD_GRAVITY)
}

/// Transmission measured at a junction: incoming pipe `b_in` (downstream end
/// at the node, carrying the wave) and undisturbed outgoing pipes.
fn measured_transmission(b_in: f64, outgoing: &[f64]) -> f64 {
    let mut arrivals = vec![Arrival {
        characteristic: incoming_c_plus(),
        impedance: b_in,
        end: PipeEnd::Downstream,
    }];
    arrivals.extend(outgoing.iter().map(|&b| Arrival {
        characteristic: H0,
        impedance: b,
        end: PipeEnd::Upstream,
    }));
    (junction::head(&arrivals, 0.0) - H0) / F
}

#[test]
fn dead_end_doubles_the_wave() {
    println!("Wave of {F} m reaching a closed end:");
    let (h, q) = dead_end::downstream(incoming_c_plus());
    check_close("head rise at the dead end [m]", h - H0, 2.0 * F, 1e-15);
    check_close("reflection R (docs: +1)", (h - H0) / F - 1.0, 1.0, 1e-15);
    check_close("flow at the dead end", q, 0.0, 0.0);
    let b = impedance(1000.0);
    check_close(
        "junction formula with one pipe gives R",
        joukowsky::junction_reflection(b, &[b]),
        1.0,
        1e-15,
    );
}

#[test]
fn reservoir_inverts_the_wave() {
    println!("Wave of {F} m reaching a large reservoir:");
    let b = impedance(1000.0);
    let (h, q) = reservoir::downstream(H0, incoming_c_plus(), b);
    check_close("head at the reservoir [m]", h, H0, 1e-15);
    check_close("reflection R (docs: -1)", (h - H0) / F - 1.0, -1.0, 1e-15);
    println!("  flow into the reservoir doubles: {q:.6} m^3/s");
    check_close("flow = 2F / B [m^3/s]", q, 2.0 * F / b, 1e-12);
}

#[test]
fn va3_tee_of_identical_pipes_transmits_two_thirds() {
    println!("Wave of {F} m reaching a tee of three identical DN150 pipes:");
    let b = impedance(1000.0);
    let t = measured_transmission(b, &[b, b]);
    check_close("transmitted T (docs: 2/3)", t, 2.0 / 3.0, 1e-12);
    check_close("reflected R (docs: -1/3)", t - 1.0, -1.0 / 3.0, 1e-12);
    check_close(
        "matches the closed-form formula",
        t,
        joukowsky::junction_transmission(b, &[b, b, b]),
        1e-12,
    );
    println!("  => each tee cuts the front by a third: waves fade fast in branched networks.");
}

#[test]
fn va3_steel_into_pvc_transmits_less_than_half() {
    println!("Wave travelling from a steel pipe into a uPVC pipe of the same bore:");
    let (b_steel, b_pvc) = (impedance(1250.0), impedance(370.0));
    println!("  B_steel / B_PVC = {:.2} (docs: ~3.4)", b_steel / b_pvc);
    let t = measured_transmission(b_steel, &[b_pvc]);
    check_close("transmitted T (docs: ~0.46)", t, 0.46, 0.02);
    check_close(
        "matches the closed-form formula",
        t,
        joukowsky::junction_transmission(b_steel, &[b_steel, b_pvc]),
        1e-12,
    );
    println!("  => material changes partly reflect waves - a feature localisation can use.");
}
