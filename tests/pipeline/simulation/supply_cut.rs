//! Supply cuts as alerts (V-S9): the estate's three loggers, live in 1 s
//! chunks. The municipal supply to two of them fails at 100 s - their
//! pressure drains away over a minute, as a pipe empties, not as a wave -
//! and comes back at 460 s with a 10 s rise; the third stays supplied.
//!
//! The engine must raise ONE supply-cut alert for the two (Warning: two
//! loggers agree), revise it when the water returns, and leave the third
//! out. The drain is slow, so no burst alarm. One logger alone is raised
//! for Review (it could be a sensor fault); a logger whose data stops while
//! its pipe is empty says so (P6).

use jk_application::{Alert, Level};
use jk_classification::{Context, EventClass};
use jk_tests::check;

use crate::{estate_site, live, record};

const DT: f64 = 0.01;

/// A logger's true pressure: 30 m; if `cut`, draining to 0 over 100-160 s
/// and back over 460-470 s; 600 s in all.
fn truth(cut: bool) -> Vec<f64> {
    (0..60_000)
        .map(|k| {
            let t = f64::from(k) * DT;
            if !cut || t < 100.0 {
                30.0
            } else if t < 160.0 {
                30.0 * (1.0 - (t - 100.0) / 60.0)
            } else if t < 460.0 {
                0.0
            } else if t < 470.0 {
                30.0 * (t - 460.0) / 10.0
            } else {
                30.0
            }
        })
        .collect()
}

fn cuts(alerts: &[&Alert]) -> Vec<Alert> {
    alerts
        .iter()
        .filter(|a| a.class == EventClass::SupplyCut)
        .map(|a| (*a).clone())
        .collect()
}

#[test]
fn vs9_a_supply_cut_is_raised_once_and_revised_when_the_water_returns() {
    let site = estate_site(Context::unknown());
    println!("Supply to S1 and S2 fails at 100 s and returns at 460 s; S3 stays supplied:");
    let truths = vec![truth(true), truth(true), truth(false)];
    let issued = live(&site, &record(&truths, DT, 51), 1.0, &[], None);
    let all: Vec<&Alert> = issued.iter().map(|i| &i.alert).collect();
    let cut = cuts(&all);
    check(
        "  one supply-cut alert, first sent once the pipes had stood empty a minute",
        cut.first().is_some_and(|a| a.revision == 0)
            && issued
                .iter()
                .find(|i| i.alert.class == EventClass::SupplyCut)
                .is_some_and(|i| (200.0..240.0).contains(&i.data_time)),
    );
    check(
        "  for S1 and S2 together, at Warning (two loggers agree), S3 not in it",
        cut.iter().all(|a| {
            a.id == cut[0].id
                && a.level == Level::Warning
                && a.sensors.len() == 2
                && !a.sensors.iter().any(|s| s.starts_with("S3"))
        }),
    );
    check(
        "  revised when the water returned, saying so",
        cut.last().is_some_and(|a| {
            a.revision >= 1
                && a.reasons
                    .iter()
                    .any(|r| r == "supply back everywhere it was lost")
        }),
    );
    check(
        "  the slow drain raised no burst alarm",
        !all.iter().any(|a| a.class == EventClass::Burst),
    );
}

#[test]
fn vs9_one_logger_alone_is_raised_for_review() {
    let site = estate_site(Context::unknown());
    println!("Only S1's pressure falls to 0 (its supply - or its transducer):");
    let truths = vec![truth(true), truth(false), truth(false)];
    let issued = live(&site, &record(&truths, DT, 52), 1.0, &[], None);
    let all: Vec<&Alert> = issued.iter().map(|i| &i.alert).collect();
    let cut = cuts(&all);
    check(
        "  a supply cut at Review, naming the sensor-fault possibility",
        cut.first().is_some_and(|a| {
            a.level == Level::Review && a.reasons.iter().any(|r| r.contains("sensor fault"))
        }),
    );
}

#[test]
fn vs9_data_stopping_while_empty_is_said() {
    let site = estate_site(Context::unknown());
    println!("Supply to S1 and S2 fails; S2's gateway goes off at 300 s, still empty:");
    let truths = vec![truth(true), truth(true), truth(false)];
    let issued = live(
        &site,
        &record(&truths, DT, 53),
        1.0,
        &[],
        Some((1, 300.0, 1e9)),
    );
    let all: Vec<&Alert> = issued.iter().map(|i| &i.alert).collect();
    let cut = cuts(&all);
    check(
        "  the alert says S2's data stopped while empty - its return not known",
        cut.iter().any(|a| {
            a.reasons
                .iter()
                .any(|r| r.starts_with("S2") && r.contains("data stopped"))
        }),
    );
}
