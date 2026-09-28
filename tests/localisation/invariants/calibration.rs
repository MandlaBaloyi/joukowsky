//! "Does the 90% region contain the truth ~90% of the time?"
//! (`validation_strategy.txt`, localisation metrics: credible-region
//! coverage). A map that is confidently wrong is worse than no map.
//!
//! Each trial: a random burst point on the estate network, true wave speeds
//! that differ from the book values by a random few percent (as real pipes
//! do), arrival times with timing noise. The localiser knows only the book
//! wave speeds and that they are uncertain by that much.

use jk_hydraulics::{Network, Node};
use jk_kernel::random::Rng;
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_localisation::{Arrival, LocaliseOptions, localise};
use jk_tests::{check, pipe};

use crate::A;

/// Estate pipes as (from, to, length); nodes 0 = R, 1 = S1, 2 = T, 3 = S2, 4 = S3.
const PIPES: [(usize, usize, f64); 4] =
    [(0, 1, 400.0), (1, 2, 300.0), (2, 3, 500.0), (2, 4, 600.0)];

fn book_network() -> Network {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let ids: Vec<_> = std::iter::once(r)
        .chain((0..4).map(|_| net.add_node(Node::Junction { demand: Flow::ZERO })))
        .collect();
    for (u, v, l) in PIPES {
        net.add_pipe(ids[u], ids[v], pipe(l, 0.15, A, 0.02))
            .unwrap();
    }
    net
}

/// Arrival times at S1 (node 1), S2 (3), S3 (4) for a burst `d` metres
/// along pipe `p` from its start, with true wave speeds `a` per pipe.
fn true_arrivals(p: usize, d: f64, a: &[f64; 4], t0: f64) -> [f64; 3] {
    // Travel time along the tree from the burst to each sensor.
    let tau = |i: usize, len: f64| len / a[i];
    let (to_s1, to_t) = match p {
        0 => (tau(0, 400.0 - d), tau(0, 400.0 - d) + tau(1, 300.0)),
        1 => (tau(1, d), tau(1, 300.0 - d)),
        2 => (tau(2, d) + tau(1, 300.0), tau(2, d)),
        _ => (tau(3, d) + tau(1, 300.0), tau(3, d)),
    };
    let s2 = if p == 2 {
        tau(2, 500.0 - d)
    } else {
        to_t + tau(2, 500.0)
    };
    let s3 = if p == 3 {
        tau(3, 600.0 - d)
    } else {
        to_t + tau(3, 600.0)
    };
    [t0 + to_s1, t0 + s2, t0 + s3]
}

fn coverage(trials: u32, speed_spread: f64) -> (f64, f64) {
    let net = book_network();
    let sensors = [1_usize, 3, 4].map(|i| net.node_id(i).unwrap());
    let mut rng = Rng::new(424_242);
    let sigma = 0.2e-3;
    let options = LocaliseOptions {
        wave_speed_uncertainty: speed_spread,
        ..LocaliseOptions::default()
    };
    let (mut inside, mut length_sum) = (0_u32, 0.0);
    for _ in 0..trials {
        // A random point, anywhere except the first 20 m of the reservoir
        // pipe (a burst there is hidden behind the reservoir's reflection).
        let p = (rng.uniform() * 4.0).floor().min(3.0);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let p = p as usize;
        let d = 20.0 + rng.uniform() * (PIPES[p].2 - 40.0);
        let a: [f64; 4] = std::array::from_fn(|_| A * (1.0 + speed_spread * rng.normal()));
        let t = true_arrivals(p, d, &a, 2.0);
        let arrivals: Vec<Arrival> = sensors
            .iter()
            .zip(t)
            .map(|(&node, ti)| Arrival {
                node,
                time: Time::new(ti + sigma * rng.normal()),
                uncertainty: Time::new(sigma),
            })
            .collect();
        let loc = localise(&net, &arrivals, &options).unwrap();
        let pipe_id = net.pipe_id(p).unwrap();
        if loc
            .credible_region
            .iter()
            .any(|s| s.pipe == pipe_id && s.from.value() - 1e-9 <= d && d <= s.to.value() + 1e-9)
        {
            inside += 1;
        }
        length_sum += loc.credible_length.value();
    }
    (
        f64::from(inside) / f64::from(trials),
        length_sum / f64::from(trials),
    )
}

#[test]
fn the_90_percent_region_holds_the_truth_about_90_percent_of_the_time() {
    let (hit, length) = coverage(60, 0.03);
    println!("60 random bursts; true wave speeds off by ~3%; timing noise 0.2 ms:");
    println!(
        "  truth inside the 90% region: {:.0}% of trials",
        hit * 100.0
    );
    println!("  average region length: {length:.0} m");
    check(
        "coverage between 75% and 99% (60 trials)",
        (0.75..=0.99).contains(&hit),
    );
}

#[test]
#[ignore = "full suite: 400 random localisations"]
fn calibration_holds_over_many_trials() {
    let (hit, length) = coverage(400, 0.03);
    println!("400 random bursts; wave speeds off by ~3%; timing noise 0.2 ms:");
    println!(
        "  truth inside the 90% region: {:.1}% (ideal 90%)",
        hit * 100.0
    );
    println!("  average region length: {length:.0} m");
    check("coverage between 85% and 95%", (0.85..=0.95).contains(&hit));
}
