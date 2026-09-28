//! A straight main between two sensors: the graph method must reproduce the
//! two-sensor formula x = (L + a (t1 - t2)) / 2, and its region must be as
//! narrow as the timing allows.
//!
//! ```text
//!   R1 --300-- S1 ------700------ burst ----------1300---------- S2 --300-- R2
//! ```

use jk_hydraulics::{Network, Node};
use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, Time};
use jk_localisation::{Arrival, LocaliseOptions, localise};
use jk_tests::{check, check_close, pipe};

use crate::A;

#[test]
fn a_burst_on_a_straight_main_is_found_where_the_formula_says() {
    let main = |l| pipe(l, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r1 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(j());
    let s2 = net.add_node(j());
    let r2 = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    net.add_pipe(r1, s1, main(300.0)).unwrap();
    let between = net.add_pipe(s1, s2, main(2000.0)).unwrap();
    net.add_pipe(s2, r2, main(300.0)).unwrap();

    let sigma = Time::new(0.15e-3); // what the detector achieved at 128 S/s
    let t0 = 2.0;
    let arrivals = [
        Arrival {
            node: s1,
            time: Time::new(t0 + 0.7),
            uncertainty: sigma,
        },
        Arrival {
            node: s2,
            time: Time::new(t0 + 1.3),
            uncertainty: sigma,
        },
    ];
    let loc = localise(&net, &arrivals, &LocaliseOptions::default()).unwrap();

    println!("Arrivals 0.7 s and 1.3 s after the burst, +/- 0.15 ms each:");
    println!(
        "  most likely: pipe {} at {:.3} m",
        loc.most_likely.pipe.index(),
        loc.most_likely.distance.value()
    );
    check(
        "on the main between the sensors",
        loc.most_likely.pipe == between,
    );
    check_close(
        "distance from S1 [m]",
        loc.most_likely.distance.value(),
        700.0,
        1e-6,
    );
    check_close("burst time recovered [s]", loc.emission.value(), t0, 1e-9);
    check_close(
        "probability on that pipe",
        loc.pipes[0].probability,
        1.0,
        1e-9,
    );
    let expected_sigma = A * sigma.value() / std::f64::consts::SQRT_2;
    println!(
        "  90% region: {:.1} m of pipe (timing alone: sigma_x = a sigma / sqrt2 = {:.2} m; grid cells are 2 m)",
        loc.credible_length.value(),
        expected_sigma
    );
    let r = loc.credible_region[0];
    check(
        "the region contains the truth",
        r.from.value() <= 700.0 && 700.0 <= r.to.value(),
    );
    check(
        "the region is at most two grid cells",
        loc.credible_length <= Length::new(4.0 + 1e-9),
    );
    check_close("misfit of a perfect fit", loc.misfit, 0.0, 0.0);
}
