//! Skipping hopeless pipes must not change any answer.
//!
//! On a large network most pipes cannot be where a burst came from: their
//! timing misses by seconds. The locator bounds the best score any point on
//! a pipe could reach and skips pipes that cannot come within e^-50 of the
//! best point found (branch and bound, `locate.rs`). These tests locate the
//! same bursts both ways - skipping, and scoring every candidate - and
//! demand the same point, search stretch, pipe probabilities and misfit.

use jk_hydraulics::{Network, NodeId};
use jk_kernel::random::Rng;
use jk_kernel::{Length, Time};
use jk_localisation::graph::TravelGraph;
use jk_localisation::{Arrival, Localisation, LocaliseOptions, Localiser};
use jk_sim::{TownOptions, town};
use jk_tests::{check, check_close};

use crate::estate;

/// Arrival times at `sensors` for a burst `fraction` along `pipe` at 1 s,
/// with Gaussian timing noise of `noise` seconds.
fn arrivals(
    net: &Network,
    sensors: &[NodeId],
    pipe: usize,
    fraction: f64,
    noise: f64,
    rng: &mut Rng,
) -> Vec<Arrival> {
    let graph = TravelGraph::new(net);
    let nominal: Vec<f64> = net
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let p = &net.pipes()[pipe];
    sensors
        .iter()
        .map(|&s| {
            let times = graph.times_from(s.index(), &nominal);
            let t = graph.point_time(
                s.index(),
                &times,
                (p.from.index(), p.to.index()),
                fraction,
                nominal[pipe],
            );
            Arrival {
                node: s,
                time: Time::new(1.0 + t + noise * rng.normal()),
                uncertainty: Time::new(1e-3),
            }
        })
        .collect()
}

fn same(fast: &Localisation, full: &Localisation) {
    check(
        "same most likely pipe",
        fast.most_likely.pipe == full.most_likely.pipe,
    );
    check_close(
        "most likely point [m]",
        fast.most_likely.distance.value(),
        full.most_likely.distance.value(),
        1e-9,
    );
    check_close(
        "90% search stretch [m]",
        fast.credible_length.value(),
        full.credible_length.value(),
        1e-9,
    );
    check_close("misfit", fast.misfit, full.misfit, 1e-9);
    let top = |l: &Localisation| l.pipes.first().map_or(0.0, |p| p.probability);
    check_close("probability of the top pipe", top(fast), top(full), 1e-9);
}

#[test]
fn skipping_changes_nothing_for_40_random_bursts_and_logger_sets() {
    let t = town(&TownOptions {
        length: Length::new(6_000.0),
        seed: 21,
        ..TownOptions::default()
    })
    .unwrap();
    let net = &t.network;
    let mut rng = Rng::new(33);
    let pick = |rng: &mut Rng, n: usize| {
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )] // in 0..n, n small
        let i = ((rng.uniform() * n as f64) as usize).min(n - 1);
        i
    };
    let mut worst: f64 = 0.0;
    println!("40 random bursts on a 6 km town, 2-6 loggers each, 0-3 ms timing noise:");
    for _ in 0..40 {
        let count = 2 + pick(&mut rng, 5);
        let mut sensors: Vec<NodeId> = Vec::new();
        while sensors.len() < count {
            let s = t.hydrants[pick(&mut rng, t.hydrants.len())];
            if !sensors.contains(&s) {
                sensors.push(s);
            }
        }
        let options = LocaliseOptions {
            spacing: Length::new(5.0),
            wave_speed_uncertainty: 0.15 * rng.uniform(),
            ..LocaliseOptions::default()
        };
        let pipe = pick(&mut rng, net.pipes().len());
        let noise = 3e-3 * rng.uniform();
        let a = arrivals(net, &sensors, pipe, rng.uniform(), noise, &mut rng);
        let l = Localiser::new(net, &sensors, &options).unwrap();
        let (fast, full) = (l.locate(&a).unwrap(), l.locate_exhaustive(&a).unwrap());
        assert_eq!(fast.most_likely.pipe, full.most_likely.pipe);
        for (x, y) in [
            (
                fast.most_likely.distance.value(),
                full.most_likely.distance.value(),
            ),
            (fast.credible_length.value(), full.credible_length.value()),
            (fast.misfit, full.misfit),
        ] {
            worst = worst.max((x - y).abs() / x.abs().max(y.abs()).max(1e-12));
        }
    }
    check_close("largest relative difference, 40 bursts", worst, 0.0, 1e-9);
}

#[test]
fn skipping_pipes_changes_nothing_on_the_estate() {
    let (e, _) = estate(None);
    let options = LocaliseOptions {
        wave_speed_uncertainty: 0.05,
        ..LocaliseOptions::default()
    };
    let l = Localiser::new(&e.net, &e.sensors, &options).unwrap();
    let mut rng = Rng::new(5);
    for (pipe, fraction) in [(3, 0.33), (2, 0.8), (0, 0.5)] {
        println!("Estate, burst {fraction} along pipe {pipe}:");
        let a = arrivals(&e.net, &e.sensors, pipe, fraction, 0.5e-3, &mut rng);
        same(&l.locate(&a).unwrap(), &l.locate_exhaustive(&a).unwrap());
    }
}

#[test]
fn skipping_pipes_changes_nothing_on_a_10_km_town() {
    let t = town(&TownOptions {
        length: Length::new(10_000.0),
        seed: 3,
        ..TownOptions::default()
    })
    .unwrap();
    let net = &t.network;
    let options = LocaliseOptions {
        spacing: Length::new(5.0),
        wave_speed_uncertainty: 0.1,
        ..LocaliseOptions::default()
    };
    let mut rng = Rng::new(9);
    let h = &t.hydrants;
    // Four loggers around town; bursts on pipes inside and outside them,
    // with clean timing and with 2 ms of timing noise (a poor clock).
    let sensors = [h[5], h[40], h[70], h[100]];
    let l = Localiser::new(net, &sensors, &options).unwrap();
    for (k, (pipe, fraction)) in [(10, 0.5), (60, 0.25), (95, 0.9), (110, 0.4)]
        .into_iter()
        .enumerate()
    {
        let noise = if k % 2 == 0 { 0.0 } else { 2e-3 };
        println!(
            "10 km town, burst {fraction} along pipe {pipe}, timing noise {} ms:",
            noise * 1e3
        );
        let a = arrivals(net, &sensors, pipe, fraction, noise, &mut rng);
        same(&l.locate(&a).unwrap(), &l.locate_exhaustive(&a).unwrap());
    }
}
