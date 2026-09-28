//! The live monitor: data arriving in pieces, as from a real site.
//!
//! A real site does not hand over whole records. Each logger sends a chunk
//! every second (or minute); connections drop; load-shedding switches
//! gateways off. The monitor must decide each event as soon as the data
//! allows, never twice, and say plainly what it could not see (P6).

use jk_application::Level;
use jk_classification::Context;
use jk_hydraulics::InitialCondition;
use jk_kernel::Head;
use jk_tests::check;
use jk_tests::estate::estate;

use super::scenarios::{flowing_two_bursts, hole};
use crate::{estate_site, live, record, run_pipeline, simulate};

/// The estate at rest (60 m), a 10 mm hole opening 200 m along the south
/// branch at `at` seconds; `duration` seconds recorded at S1, S2, S3.
fn one_burst(at: f64, duration: f64, seed: u64) -> Vec<crate::Recorded> {
    let (sim, b) = estate(Some(200.0));
    let mut net = sim.net;
    net.add_burst(hole(b.unwrap(), at)).unwrap();
    let rest = InitialCondition::AtRest {
        head: Head::new(60.0),
    };
    let (truths, dt) = simulate(&net, &rest, &sim.sensors, duration);
    record(&truths, dt, seed)
}

#[test]
fn live_feed_gives_the_same_alerts_as_whole_records_within_seconds() {
    let (truths, dt, site) = flowing_two_bursts();
    println!("Two bursts (t = 3 s, t = 30 s) on the flowing estate.");
    println!("\nWhole records, for comparison:");
    let whole = run_pipeline(&site, &truths, dt, 3);
    println!("\nLive: each sensor sends a 1 s chunk in turn:");
    let issued = live(&site, &record(&truths, dt, 3), 1.0, &[], None);
    let alerts: Vec<_> = issued.iter().map(|i| i.alert.clone()).collect();
    check("the same alerts as from whole records", alerts == whole);
    // How soon: classification looks 3 s past the first front; the other
    // sensors must have reported past the time the wave reaches them; the
    // data comes in 1 s pieces. So ~4-5 s of data time after the burst.
    for (i, burst) in [3.0, 30.0].iter().enumerate() {
        let delay = issued[i].data_time - burst;
        println!(
            "  alert {} went out {delay:.2} s (data time) after its burst",
            i + 1
        );
        check("  ... within 5.5 s", delay < 5.5);
    }
}

#[test]
fn a_sensor_that_comes_back_late_revises_the_alert() {
    let site = estate_site(Context::unknown());
    let recorded = one_burst(3.0, 20.0, 5);
    println!("A burst at t = 3 s. S2's connection is down; its data arrives only");
    println!("after everything else. The monitor waits 10 s (data time) for it, then");
    println!("decides without it - and revises when S2's data turns up:");
    let issued = live(&site, &recorded, 1.0, &[1], None);
    check(
        "two alerts: the first issue and one revision",
        issued.len() == 2,
    );
    let (first, second) = (&issued[0].alert, &issued[1].alert);
    check(
        "first: alert #1, revision 0",
        first.id == 1 && first.revision == 0,
    );
    check("  seen by S1 and S3 only", first.sensors.len() == 2);
    check(
        "  says S2 sent nothing",
        first
            .reasons
            .iter()
            .any(|r| r.starts_with("S2 north block: no data")),
    );
    check("  still an ALARM, located", first.level == Level::Alarm);
    check(
        "then: alert #1 again, revision 1",
        second.id == 1 && second.revision == 1,
    );
    check("  now seen by all three sensors", second.sensors.len() == 3);
    let loc = second.location.as_ref().expect("located");
    check(
        "  on the south branch within 2 m of the truth",
        loc.most_likely.pipe.index() == 3 && (loc.most_likely.distance.value() - 200.0).abs() < 2.0,
    );
}

#[test]
fn a_power_cut_at_a_sensor_is_reported_not_taken_for_silence() {
    let site = estate_site(Context::unknown());
    let recorded = one_burst(3.0, 15.0, 6);
    println!("A burst at t = 3 s while load-shedding has cut S2's gateway from");
    println!("t = 2.5 s to t = 6 s: S2's samples for that time never arrive, and when");
    println!("it comes back the network is still ringing:");
    let issued = live(&site, &recorded, 1.0, &[], Some((1, 2.5, 6.0)));
    check(
        "exactly one alert - no echo alerts from S2 coming back",
        issued.len() == 1,
    );
    let a = &issued[0].alert;
    check(
        "an ALARM, seen by S1 and S3",
        a.level == Level::Alarm && a.sensors.len() == 2,
    );
    check(
        "says S2's silence means nothing",
        a.reasons
            .iter()
            .any(|r| r.starts_with("S2 north block was not watching")),
    );
    let loc = a.location.as_ref().expect("located");
    check(
        "located on the south branch within 2 m",
        loc.most_likely.pipe.index() == 3 && (loc.most_likely.distance.value() - 200.0).abs() < 2.0,
    );
}
