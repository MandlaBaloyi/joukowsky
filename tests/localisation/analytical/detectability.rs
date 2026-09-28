//! Would a burst be FELT at the loggers? The first front, by hand.
//!
//! ```text
//!   R --400-- S1 --300-- T --500-- S2 (dead end)
//!                        |
//!                       600    a sudden 1 l/s burst 200 m from T
//!                        |
//!                        S3 (dead end)
//! ```
//!
//! All pipes 150 mm, a = 1000 m/s: impedance B = a / (g A) = 5 768 s/m^2.
//! The burst drops the head at the hole by h0 = (B / 2) Q = 2.884 m and
//! sends that drop both ways. Then, junction by junction
//! (`friction_and_damping.txt` section 4; T = 2/3 at a tee of equal pipes,
//! 1 on a straight run, 2 at a dead end):
//!
//! ```text
//!   S3: 400 m of straight pipe, then the dead end       2   h0 = 5.77 m
//!   T : the tee                                         2/3 h0 = 1.92 m
//!   S1: the tee, then a straight run (R-S1 = S1-T)      2/3 h0 = 1.92 m
//!   S2: the tee, then the dead end                      4/3 h0 = 3.85 m
//!   R : a reservoir holds its head                      0
//! ```

use jk_kernel::{Area, Flow, Length, STANDARD_GRAVITY};
use jk_localisation::Point;
use jk_localisation::placement::{PlacementOptions, coverage, detectability, first_fronts};
use jk_tests::{check, check_close};

use crate::{A, estate};

#[test]
fn the_first_front_at_each_logger_follows_the_junction_rules() {
    let (e, _) = estate(None);
    let [s1, s2, s3] = e.sensors;
    let reservoir = e.net.node_id(0).unwrap();
    let point = Point {
        pipe: e.pipes[3],
        distance: Length::new(200.0),
    };
    let heads = first_fronts(
        &e.net,
        &[s3, e.tee, s1, s2, reservoir],
        point,
        Flow::new(1e-3),
    )
    .unwrap();
    let b = A / (STANDARD_GRAVITY.value() * Area::of_circle(Length::new(0.15)).value());
    let h0 = 0.5 * b * 1e-3;
    println!("A sudden 1 l/s burst drops the head at the hole by h0 = {h0:.3} m.");
    println!("First front felt at each logger:");
    check_close(
        "S3 (dead end, no junction)       2 h0",
        heads[0].value(),
        2.0 * h0,
        1e-12,
    );
    check_close(
        "T  (the tee)                   2/3 h0",
        heads[1].value(),
        2.0 / 3.0 * h0,
        1e-12,
    );
    check_close(
        "S1 (tee, straight run)         2/3 h0",
        heads[2].value(),
        2.0 / 3.0 * h0,
        1e-12,
    );
    check_close(
        "S2 (tee, then dead end)        4/3 h0",
        heads[3].value(),
        4.0 / 3.0 * h0,
        1e-12,
    );
    check_close(
        "R  (reservoir holds its head)       0",
        heads[4].value(),
        0.0,
        1e-12,
    );
}

#[test]
fn the_smallest_burst_three_loggers_can_locate_on_the_estate() {
    // From ANY pipe of the estate, the second-strongest first front at
    // S1, S2, S3 is 4/3 h0 (through the tee to a dead end; or straight
    // through S1, which transmits 1). Two loggers must feel it: the
    // smallest locatable burst is the threshold (6 x 2 cm = 12 cm) over
    // 4/3 x B/2 = 3 846 m per m^3/s: 0.0312 l/s, the same on every pipe.
    let (e, _) = estate(None);
    let options = PlacementOptions::default();
    let d = detectability(&e.net, &e.sensors, &options).unwrap();
    let b = A / (STANDARD_GRAVITY.value() * Area::of_circle(Length::new(0.15)).value());
    let expected = 6.0 * 0.02 / (4.0 / 3.0 * 0.5 * b);
    println!("Smallest sudden burst located anywhere on each pipe (2 cm logger noise):");
    for (p, name) in d.pipes.iter().zip(["R-S1", "S1-T", "T-S2", "T-S3"]) {
        check_close(
            &format!("  {name} [l/s]"),
            p.smallest_locatable.value() * 1000.0,
            expected * 1000.0,
            1e-9,
        );
    }
    check(
        "a 1 l/s burst is located everywhere",
        (d.locatable - 1.0).abs() < 1e-12,
    );

    // Below it, the advisor must not count loggers that cannot feel it:
    // every burst of 0.02 l/s is scored as "could be anywhere".
    let small = PlacementOptions {
        burst: Some(Flow::new(0.02e-3)),
        ..options
    };
    let geometry = PlacementOptions {
        burst: None,
        ..options
    };
    let felt = coverage(&e.net, &e.sensors, None, &small).unwrap();
    let ideal = coverage(&e.net, &e.sensors, None, &geometry).unwrap();
    println!(
        "Average 90% region, 3 loggers: {:.0} m if every burst were felt; {:.0} m for 0.02 l/s bursts",
        ideal.expected_region.value(),
        felt.expected_region.value()
    );
    check(
        "0.02 l/s bursts: blind everywhere (region = the whole 1.8 km)",
        (felt.expected_region.value() - 1800.0).abs() < 1e-9,
    );
}
