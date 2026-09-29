//! Planning zone by zone: the zones must split the network cleanly, and
//! the plan must stay close to planning the whole network at once.
//!
//! Zones make the placement advisor fast on large networks (each search is
//! local), at a price: a zone planned early cannot count on loggers its
//! neighbours place later - a whole-town polish repairs most of it. These
//! tests measure what is left (FN-020).

use jk_kernel::Length;
use jk_localisation::placement::{PlacementOptions, advise, advise_zoned, zones};
use jk_sim::{TownOptions, town};
use jk_tests::{check, check_close};

fn a_town(km: f64, seed: u64) -> jk_sim::Town {
    town(&TownOptions {
        length: Length::new(km * 1000.0),
        seed,
        ..TownOptions::default()
    })
    .unwrap()
}

#[test]
fn zones_cover_every_pipe_exactly_once() {
    let t = a_town(20.0, 4);
    let split = zones(&t.network, Length::new(5_000.0), 100).unwrap();
    let pipes = t.network.pipes();
    let mut seen = vec![0usize; pipes.len()];
    for z in &split {
        for &p in z {
            seen[p] += 1;
        }
    }
    let lengths: Vec<f64> = split
        .iter()
        .map(|z| {
            z.iter()
                .map(|&p| pipes[p].pipe.length().value())
                .sum::<f64>()
                / 1000.0
        })
        .collect();
    println!("20 km town in zones of about 5 km: {lengths:.2?} km");
    check("4 zones", split.len() == 4);
    check(
        "every pipe in exactly one zone",
        seen.iter().all(|&n| n == 1),
    );
    check(
        "no zone more than twice the average",
        lengths.iter().all(|&l| l <= 2.0 * 20.0 / 4.0),
    );
}

#[test]
fn one_zone_gives_exactly_the_plain_advice() {
    let t = a_town(3.0, 2);
    let options = PlacementOptions::default();
    let whole = advise(&t.network, &t.hydrants, 3, &[], None, &options).unwrap();
    let zoned = advise_zoned(
        &t.network,
        &t.hydrants,
        3,
        &[],
        None,
        &options,
        Length::new(10_000.0),
    )
    .unwrap();
    println!(
        "3 km town, 3 loggers: whole {:?}, zoned {:?}",
        whole.best.sensors, zoned.best.sensors
    );
    check("one zone", zoned.zones.len() == 1);
    check("the same loggers", whole.best.sensors == zoned.best.sensors);
    check_close(
        "the same mean 90% region [m]",
        zoned.best.expected_region.value(),
        whole.best.expected_region.value(),
        1e-12,
    );
}

#[test]
#[ignore = "full suite: plans a 5 km town twice (about a minute in debug)"]
fn zoned_plans_stay_close_to_whole_town_plans() {
    let t = a_town(5.0, 6);
    let options = PlacementOptions::default();
    let count = 6;
    let whole = advise(&t.network, &t.hydrants, count, &[], None, &options).unwrap();
    let zoned = advise_zoned(
        &t.network,
        &t.hydrants,
        count,
        &[],
        None,
        &options,
        Length::new(2_000.0),
    )
    .unwrap();
    let (w, z) = (
        whole.best.expected_region.value(),
        zoned.best.expected_region.value(),
    );
    println!(
        "5 km town, {count} loggers: whole town at once {w:.1} m mean 90% region ({:.0}% pinpointed); \
         {} zones {z:.1} m ({:.0}% pinpointed)",
        whole.best.pinpointed * 100.0,
        zoned.zones.len(),
        zoned.best.pinpointed * 100.0
    );
    check("planned in more than one zone", zoned.zones.len() > 1);
    check(
        "the asked number of loggers",
        zoned.best.sensors.len() == count,
    );
    check("within 10% of the whole-town plan", z <= 1.1 * w);
}
