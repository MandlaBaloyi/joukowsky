//! The engine end to end on a branched estate network:
//! physics -> loggers -> detection -> localisation.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2
//!                        |
//!                       600   <- a 10 mm hole opens 200 m from T, t = 2.0 s
//!                        |
//!                        S3
//! ```
//!
//! 1. Hydraulics simulates the burst every millisecond.
//! 2. Three synthetic 128 S/s loggers (random clock phase, 2 cm noise)
//!    record S1, S2 and S3.
//! 3. Detection finds the front in each record and times it.
//! 4. Localisation, knowing only the pipe map and the three arrival times,
//!    says where the burst is.

use jk_detection::{Detector, DetectorConfig, Series};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Burst, BurstOpening, GridOptions, InitialCondition, Probe};
use jk_kernel::{Area, Head, Length, STANDARD_GRAVITY, Time};
use jk_localisation::{Arrival, LocaliseOptions, localise};
use jk_tests::sensor::{Logger, Rng, sample};
use jk_tests::{check, check_close};

use crate::estate;

#[test]
#[allow(clippy::too_many_lines)] // the whole chain, told as one story
fn a_burst_on_a_branch_is_found_from_raw_logger_data() {
    // 1. Physics: the same estate, with the branch split where the pipe fails.
    let (sim, burst_node) = estate(Some(200.0));
    let mut net = sim.net;
    net.add_burst(Burst {
        node: burst_node.expect("split"),
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
            Time::new(3.3),
            &sim.sensors.map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 200, // 200 m pipe in 200 reaches: dt = 1 ms
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();

    // 2 + 3. Loggers and detection at each sensor.
    let mut rng = Rng::new(7);
    let detector = Detector::new(DetectorConfig {
        calibration: Time::new(1.0),
        ..DetectorConfig::default()
    })
    .unwrap();
    let mut arrivals = Vec::new();
    for (i, name) in ["S1", "S2", "S3"].iter().enumerate() {
        let truth: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
        let logger = Logger {
            rate: 128.0,
            phase: rng.uniform() / 128.0,
            noise: 0.02,
        };
        let samples = sample(&truth, dt, &logger, &mut rng);
        let found = detector
            .run(&Series {
                samples: &samples,
                start: Time::new(logger.phase),
                interval: Time::new(1.0 / 128.0),
            })
            .unwrap();
        let e = found
            .events
            .first()
            .copied()
            .expect("the burst must be detected at every sensor");
        println!(
            "  {name}: front at {:.5} s (+/- {:.2} ms), step {:+.2} m",
            e.arrival.value(),
            e.arrival_uncertainty.value() * 1e3,
            e.step.value()
        );
        arrivals.push(Arrival {
            node: sim.sensors[i],
            time: e.arrival,
            uncertainty: e.arrival_uncertainty,
        });
    }

    // 4. Localisation on the plain pipe map (the engine does not know where
    //    the split is - that is what it must find).
    let (map, _) = estate(None);
    let loc = localise(&map.net, &arrivals, &LocaliseOptions::default()).unwrap();
    println!("Localisation:");
    for p in loc.pipes.iter().take(3) {
        println!(
            "  pipe {} holds {:.1}% of the probability",
            p.pipe.index(),
            p.probability * 100.0
        );
    }
    println!(
        "  most likely: pipe {} at {:.2} m from its start (true: branch T-S3, 200 m)",
        loc.most_likely.pipe.index(),
        loc.most_likely.distance.value()
    );
    println!(
        "  90% region: {:.1} m of pipe; timing misfit {:.2}",
        loc.credible_length.value(),
        loc.misfit
    );
    println!(
        "  burst emitted at {:.4} s (true 2.0 s)",
        loc.emission.value()
    );
    check(
        "the right branch is the most likely pipe",
        loc.most_likely.pipe == map.pipes[3],
    );
    check_close(
        "distance from T [m]",
        loc.most_likely.distance.value(),
        200.0,
        2.0 / 200.0,
    );
    check(
        "the 90% region contains the truth",
        loc.credible_region
            .iter()
            .any(|s| s.pipe == map.pipes[3] && s.from.value() <= 200.0 && 200.0 <= s.to.value()),
    );
    check("the arrivals fit one source (misfit < 3)", loc.misfit < 3.0);
}
