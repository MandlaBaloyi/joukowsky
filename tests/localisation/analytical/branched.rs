//! A branched network: when the sensors CANNOT decide, the engine must say
//! so (principle P3) - and a well-placed extra sensor must fix it.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2
//!                        |
//!                       600    (burst 200 m from T)
//!                        |
//!                        S3
//! ```
//!
//! With S1 and S2 only: every point on branch T-S3 is the same extra
//! distance d from both of them, so t1 - t2 = -200/a for ANY d. The whole
//! branch explains the data equally well. With S3 as well, the burst is
//! pinned down.

use jk_kernel::{Length, Time};
use jk_localisation::{Arrival, LocaliseOptions, localise};
use jk_tests::{check, check_close};

use crate::{arrivals_on_branch, estate};

#[test]
fn two_sensors_see_a_whole_branch_three_sensors_see_the_point() {
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let branch = e.pipes[3];
    let sigma = Time::new(0.15e-3);
    let t = arrivals_on_branch(200.0, 2.0);
    let arrival = |node, time| Arrival {
        node,
        time: Time::new(time),
        uncertainty: sigma,
    };
    let options = LocaliseOptions::default();

    let two = localise(&e.net, &[arrival(s1, t[0]), arrival(s2, t[1])], &options).unwrap();
    let on_branch = two
        .pipes
        .iter()
        .find(|p| p.pipe == branch)
        .map_or(0.0, |p| p.probability);
    println!("Sensors S1 and S2 only (burst 200 m along branch T-S3):");
    println!("  probability on branch T-S3: {:.1}%", on_branch * 100.0);
    println!("  90% region: {:.0} m of pipe", two.credible_length.value());
    check("the probability sits on the right branch", on_branch > 0.99);
    check(
        "but spreads along it - the engine does not pretend to know where (region > 400 m)",
        two.credible_length > Length::new(400.0),
    );

    let three = localise(
        &e.net,
        &[arrival(s1, t[0]), arrival(s2, t[1]), arrival(s3, t[2])],
        &options,
    )
    .unwrap();
    println!("Adding sensor S3 at the end of the branch:");
    println!(
        "  most likely: branch T-S3 at {:.3} m from T; 90% region {:.1} m",
        three.most_likely.distance.value(),
        three.credible_length.value()
    );
    check(
        "the most likely pipe is the branch",
        three.most_likely.pipe == branch,
    );
    check_close(
        "distance from T [m]",
        three.most_likely.distance.value(),
        200.0,
        1e-6,
    );
    check(
        "the region shrinks to a few metres",
        three.credible_length < Length::new(5.0),
    );
    println!("  => sensor PLACEMENT decides what the engine can see (docs: placement advisor).");
}
