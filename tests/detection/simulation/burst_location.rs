//! Study V-S1: from raw logger samples to a located, sized burst.
//!
//! The Midrand example, end to end:
//!
//! ```text
//!   R1 --300 m-- S1 ------700 m------ BURST -----------1300 m----------- S2 --300 m-- R2
//! ```
//!
//! 1. The hydraulics solver simulates a 10 mm hole opening at t = 2.0 s in a
//!    DN150 main resting at 60 m, every millisecond.
//! 2. Two synthetic loggers (128 S/s, integrating, each with its own random
//!    clock phase, 2 cm noise) record the pressure at S1 and S2.
//! 3. The detector finds the event in each record and times its arrival.
//! 4. Location from the two arrival times alone: x = (L + a (t1 - t2)) / 2;
//!    size from the step: `Q_L` = 2 g A |dH| / a.
//!
//! Clocks are assumed GNSS-synchronised (docs: `time_synchronisation.txt`);
//! clock error is the next part of V-S1.

use std::fmt::Write as _;

use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, Probe, joukowsky,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time, WaveSpeed};
use jk_tests::sensor::{Logger, Rng, sample};
use jk_tests::{check, pipe};

use crate::detect;

const A: f64 = 1000.0;
const L_BETWEEN: f64 = 2000.0;
const TRUE_X: f64 = 700.0;

/// Simulated head at S1, S2 and the burst outflow, every millisecond.
struct Truth {
    s1: Vec<f64>,
    s2: Vec<f64>,
    burst_flow: Vec<f64>,
    dt: f64,
}

fn simulate() -> Truth {
    let main = |length| pipe(length, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let r1 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(Node::Junction { demand: Flow::ZERO });
    let b = net.add_node(Node::Junction { demand: Flow::ZERO });
    let s2 = net.add_node(Node::Junction { demand: Flow::ZERO });
    let r2 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    net.add_pipe(r1, s1, main(300.0)).unwrap();
    net.add_pipe(s1, b, main(TRUE_X)).unwrap();
    net.add_pipe(b, s2, main(L_BETWEEN - TRUE_X)).unwrap();
    net.add_pipe(s2, r2, main(300.0)).unwrap();
    net.add_burst(Burst {
        node: b,
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(2.0),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(4.0),
            &[Probe::Node(s1), Probe::Node(s2), Probe::Node(b)],
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 300, // dt = 1 ms
            },
        )
        .unwrap();
    let t = &run.trace;
    let heads = |i: usize| t.head(i).iter().map(|h| h.value()).collect::<Vec<_>>();
    Truth {
        s1: heads(0),
        s2: heads(1),
        burst_flow: t.flow(2).iter().map(|q| q.value()).collect(),
        dt: t.time_step().value(),
    }
}

/// Records both sensors with the given logger settings and locates/sizes
/// the burst. Returns (location error [m], sizing error [fraction]).
fn locate(truth: &Truth, rate: f64, noise: f64, rng: &mut Rng, verbose: bool) -> (f64, f64) {
    let logger = |rng: &mut Rng| Logger {
        rate,
        phase: rng.uniform() / rate,
        noise,
    };
    let (l1, l2) = (logger(rng), logger(rng));
    let x1 = sample(&truth.s1, truth.dt, &l1, rng);
    let x2 = sample(&truth.s2, truth.dt, &l2, rng);
    let (d1, d2) = (detect(&x1, rate, l1.phase), detect(&x2, rate, l2.phase));
    assert_eq!(
        d1.events.len(),
        1,
        "one event at S1 ({rate} S/s, {noise} m)"
    );
    assert_eq!(
        d2.events.len(),
        1,
        "one event at S2 ({rate} S/s, {noise} m)"
    );
    let (e1, e2) = (d1.events[0], d2.events[0]);
    let (t1, t2) = (e1.arrival.value(), e2.arrival.value());
    #[allow(clippy::manual_midpoint)] // written as in the docs
    let x = (L_BETWEEN + A * (t1 - t2)) / 2.0;
    let sized = joukowsky::burst_flow_from_head_drop(
        e1.step,
        WaveSpeed::new(A),
        Area::of_circle(Length::new(0.15)),
        STANDARD_GRAVITY,
    )
    .value();
    let lost = truth.burst_flow[truth.burst_flow.len() / 2 + 200]; // a few tenths after
    if verbose {
        println!(
            "  S1: noise learned {:.4} m, arrival {t1:.5} s (+/- {:.2} ms), step {:.3} m, alarm {:.1} ms later",
            d1.noise.value(),
            e1.arrival_uncertainty.value() * 1e3,
            e1.step.value(),
            (e1.alarm.value() - t1) * 1e3
        );
        println!(
            "  S2: noise learned {:.4} m, arrival {t2:.5} s (+/- {:.2} ms), step {:.3} m",
            d2.noise.value(),
            e2.arrival_uncertainty.value() * 1e3,
            e2.step.value()
        );
        println!("  burst located {x:.2} m from S1 (true {TRUE_X} m)");
        println!(
            "  burst sized {:.3} l/s (true {:.3} l/s)",
            sized * 1e3,
            lost * 1e3
        );
    }
    ((x - TRUE_X).abs(), (sized - lost).abs() / lost)
}

#[test]
fn vs1_burst_located_from_128_samples_per_second_loggers() {
    let truth = simulate();
    println!("DN150 main at rest (60 m); 10 mm hole opens at t = 2.0 s, 700 m from S1.");
    println!("Loggers: 128 S/s, integrating, random clock phase, 2 cm noise:");
    let (err_x, err_q) = locate(&truth, 128.0, 0.02, &mut Rng::new(1), true);
    println!(
        "  location error {err_x:.3} m, sizing error {:.2}%",
        err_q * 100.0
    );
    check(
        "located within 1 m (docs target: metre-level at 128 S/s)",
        err_x < 1.0,
    );
    check("sized within 2%", err_q < 0.02);
}

#[test]
#[ignore = "full suite: 150 recorded sensor pairs across rates and noise levels"]
fn vs1_location_error_versus_sampling_rate_and_noise() {
    let truth = simulate();
    let trials = 10;
    println!("RMS location error [m] over {trials} random clock phases / noise draws:");
    println!("  rate [S/s]   noise 0.5 cm   noise 2 cm   noise 10 cm");
    let mut rng = Rng::new(2026);
    let mut at_128_2cm = f64::INFINITY;
    for rate in [32.0, 64.0, 128.0, 256.0, 1000.0] {
        let mut row = format!("  {rate:>9}");
        for noise in [0.005, 0.02, 0.1] {
            let mut sum_sq = 0.0;
            for _ in 0..trials {
                let (e, _) = locate(&truth, rate, noise, &mut rng, false);
                sum_sq += e * e;
            }
            let rms = (sum_sq / f64::from(trials)).sqrt();
            #[allow(clippy::float_cmp)]
            if rate == 128.0 && noise == 0.02 {
                at_128_2cm = rms;
            }
            write!(row, "   {rms:>11.3}").expect("writing to a String cannot fail");
        }
        println!("{row}");
    }
    println!(
        "  (clocks perfect here; a clock error sigma_c at each sensor adds about a sigma_c / sqrt(2) in quadrature)"
    );
    check(
        "128 S/s with 2 cm noise locates to under 1 m RMS",
        at_128_2cm < 1.0,
    );
}
