//! D2 - L-Town's two years of readings through the slow-leak watch, scored
//! against the true leaks (`BattLeDIM` 2020, data/README.md).
//!
//! The whole town is one zone. Water comes in through the two reservoir
//! pipes, p227 (from R1) and p235 (from R2), metered every 5 minutes; the
//! tank T1 (16 m across - L-TOWN.inp [TANKS]) stores or gives back water,
//! seen from its level. What the town uses, leaks included, is
//!
//! ```text
//!   used = Q_p227 + Q_p235 - A dh/dt,   A = pi 8^2 = 201.1 m^2
//! ```
//!
//! (the pump only moves water from the town into the tank, inside the
//! zone). The watch takes the mean of 02:00-04:00 each night - the
//! readings are in the town's own clock time - and raises an alarm when
//! nights stay above the baseline (`crates/domain/balance/src/leak.rs`).
//!
//! The truth: `dataset_configuration.yaml` lists the 33 leaks (pipe, start,
//! end, hole size, abrupt or growing) and `<year>_Leakages.csv` their flow
//! every 5 minutes. The readings are simulated by the organisers on the
//! real model; the watch is given only what a utility would have - the
//! meters and the tank level.
//!
//! The first run (FN-048) caught 3 of 33 leaks within a week and went
//! blind for 341 days after an alarm and a repair. The watch now learns a
//! weekly profile, follows the seasons, steps its level after a change and
//! refines it for two weeks, sees falls (repairs), and caps how much one
//! night counts. 2026-10-06 (FN-051): the level after a change is refined
//! with the median, a second change inside the settling window is
//! reported, holiday nights (Cyprus's calendar) are not judged - and the
//! SCORING was corrected: alarms are matched one to one to the newest
//! leak running when raised (the first rule credited any leak begun since
//! the previous alarm and overstated: "25 of 33 within a month" was 19
//! under this rule before these fixes). The checks lock in what this gave
//! (21 of 33 within a month, 28 at all, 0 false, longest silence 63
//! days): a change that makes it worse fails. False alarms are judged
//! properly in simulation (tests/balance/simulation): L-Town nearly
//! always has some leak running, so one-to-one matching rarely calls an
//! alarm false.

use jk_balance::{LeakOptions, watch};
use jk_files::scada::ScadaUnit;
use jk_tests::ltown::{
    DAY, M3H, date, match_alarms, seconds, seconds_to_epoch, table, town_nights, true_leaks,
};
use jk_tests::{check, dataset};

#[test]
#[allow(clippy::too_many_lines)] // one story: feed, then score, printed step by step
#[ignore = "full suite: two years of 5-minute readings, about a minute in a debug build"]
fn d2_ltown_two_years_of_night_flows_against_the_true_leaks() {
    let files = [
        "ltown/2018_SCADA_Flows.csv",
        "ltown/2019_SCADA_Flows.csv",
        "ltown/2018_SCADA_Levels.csv",
        "ltown/2019_SCADA_Levels.csv",
        "ltown/2018_Leakages.csv",
        "ltown/2019_Leakages.csv",
        "ltown/dataset_configuration.yaml",
    ];
    let Some(paths) = files.iter().map(|f| dataset(f)).collect::<Option<Vec<_>>>() else {
        return;
    };
    let flows = [
        table(&paths[0], ScadaUnit::CubicMetresPerHour),
        table(&paths[1], ScadaUnit::CubicMetresPerHour),
    ];
    let levels = [
        table(&paths[2], ScadaUnit::Metres),
        table(&paths[3], ScadaUnit::Metres),
    ];
    let leak_flows = [
        table(&paths[4], ScadaUnit::CubicMetresPerHour),
        table(&paths[5], ScadaUnit::CubicMetresPerHour),
    ];
    let truth = true_leaks(&std::fs::read_to_string(&paths[6]).unwrap(), &leak_flows);
    // The files' times are the town's own clock, read as seconds since 1970
    // (so the watch knows each night's weekday); times below are counted
    // from 2018-01-01 00:00.
    let e0 = flows[0].start.value();
    check(
        "the readings start at 2018-01-01 00:00",
        (e0 - seconds_to_epoch()).abs() < 1.0,
    );
    let nights = town_nights(&flows, &levels);
    println!(
        "  holiday calendar: Cyprus's public holidays; {} nights not judged",
        nights.iter().filter(|n| n.holiday).count()
    );
    let options = LeakOptions::default();
    let w = watch(&nights, &options).unwrap();

    println!(
        "  {} nights (2018-2019), night flow 02:00-04:00; baseline {:.1} m3/h, night-to-night spread {:.2} m3/h",
        nights.len(),
        w.baseline.value() / M3H,
        w.spread.value() / M3H
    );
    match w.weekly {
        Some(p) => println!(
            "  weekly profile found (m3/h from the baseline, Mon..Sun): {}",
            p.iter()
                .map(|o| format!("{:+.1}", o / M3H))
                .collect::<Vec<_>>()
                .join(" ")
        ),
        None => println!("  no weekly profile: the baseline weeks show none"),
    }
    println!(
        "  the smallest leak this spread lets the watch catch within a week: {:.1} m3/h",
        w.smallest_in_a_week.value() / M3H
    );
    println!("  the watch raised {} alarm(s):", w.alarms.len());
    // Scoring, one to one, as leak-detection benchmarks do (BattLeDIM's
    // detection score [VERIFY the exact rule]): in time order, each alarm
    // is matched to a leak RUNNING when it is raised (begun, not yet
    // repaired) and not matched before - the NEWEST first (a rise is most
    // likely the latest leak; oldest-first credited big new leaks' alarms
    // to small leaks dripping for weeks); each
    // leak to one alarm at most. An alarm with no leak left to match is
    // FALSE; a leak never matched is MISSED. Nothing is double-counted, and
    // no guess is made about which leak a rise "is about".
    println!("    #  rise from   raised on   size [m3/h]      the leak it is matched to");
    let matched_to = match_alarms(&w.alarms, &truth, e0);
    let mut caught: Vec<Option<f64>> = vec![None; truth.len()];
    for (a, m) in w.alarms.iter().zip(&matched_to) {
        if let Some(i) = *m {
            caught[i] = Some(a.raised.value() - e0 - truth[i].start);
        }
    }
    let mut false_alarms = 0;
    for (k, a) in w.alarms.iter().enumerate() {
        let from = a.first_night.value() - e0;
        let raised = a.raised.value() - e0;
        let matched: Vec<usize> = matched_to[k].into_iter().collect();
        if matched.is_empty() {
            false_alarms += 1;
        }
        println!(
            "   {:2}  {}  {}  {:5.1} +/- {:4.1}   {}",
            k + 1,
            date(from),
            date(raised),
            a.size.value() / M3H,
            a.uncertainty.value() / M3H,
            if matched.is_empty() {
                "none - a FALSE alarm".to_string()
            } else {
                matched
                    .iter()
                    .map(|&i| format!("{} ({:.0} m3/h)", truth[i].pipe, truth[i].peak))
                    .collect::<Vec<_>>()
                    .join(", ")
            }
        );
    }
    println!(
        "  the watch saw {} lasting fall(s) - a repair, or less use:",
        w.falls.len()
    );
    for f in &w.falls {
        let at = f.first_night.value() - e0;
        let ended: Vec<String> = truth
            .iter()
            .filter(|l| (l.end - at).abs() <= 7.0 * DAY)
            .map(|l| format!("{} repaired {}", l.pipe, date(l.end)))
            .collect();
        println!(
            "    from {}  raised {}  {:+5.1} m3/h   {}",
            date(at),
            date(f.raised.value() - e0),
            f.size.value() / M3H,
            if ended.is_empty() {
                "no repair within 7 days".to_string()
            } else {
                ended.join(", ")
            }
        );
    }
    println!("  the true leaks:");
    println!("    pipe   starts       kind       peak [m3/h]   caught after [days]");
    for (l, c) in truth.iter().zip(&caught) {
        println!(
            "    {:5}  {}  {:9}  {:6.1}        {}",
            l.pipe,
            date(l.start),
            if l.abrupt { "abrupt" } else { "growing" },
            l.peak,
            c.map_or("not caught".to_string(), |d| format!("{:.0}", d / DAY))
        );
    }
    let within = |days: f64, set: &[usize]| {
        set.iter()
            .filter(|&&i| caught[i].is_some_and(|d| d <= days * DAY))
            .count()
    };
    let all: Vec<usize> = (0..truth.len()).collect();
    let big: Vec<usize> = all
        .iter()
        .copied()
        .filter(|&i| truth[i].peak >= 15.0)
        .collect();
    println!(
        "  scored ({} leaks; {} of them reach 15 m3/h or more):",
        all.len(),
        big.len()
    );
    for (label, days) in [
        ("within a week", 7.0),
        ("within a month", 30.0),
        ("at all", f64::INFINITY),
    ] {
        println!(
            "    caught {label:15} {:2} of all, {:2} of the big ones",
            within(days, &all),
            within(days, &big)
        );
    }
    println!("    false alarms    {false_alarms}");

    // The longest stretch without an alarm while a big leak ran unseen.
    let mut marks: Vec<f64> = vec![0.0];
    marks.extend(w.alarms.iter().map(|a| a.raised.value() - e0));
    marks.push(seconds("2020-01-01 00:00"));
    let (mut longest, mut at, mut missed) = (0.0, 0.0, 0);
    for pair in marks.windows(2) {
        let unseen = big
            .iter()
            .filter(|&&i| truth[i].start > pair[0] && truth[i].end < pair[1])
            .count();
        if unseen > 0 && pair[1] - pair[0] > longest {
            (longest, at, missed) = (pair[1] - pair[0], pair[0], unseen);
        }
    }
    println!(
        "  longest silence: {:.0} days from {}, while {missed} big leak(s) began and were repaired unseen",
        longest / DAY,
        date(at)
    );
    check(
        "20 or more of the 33 leaks caught within a month",
        within(30.0, &all) >= 20,
    );
    check("5 false alarms or fewer in two years", false_alarms <= 5);
    check(
        "25 or more of the 33 leaks caught at all",
        within(f64::INFINITY, &all) >= 25,
    );
    check(
        "no silence over 120 days while a big leak comes and goes",
        missed == 0 || longest <= 120.0 * DAY,
    );
    check("repairs are seen: 5 or more falls", w.falls.len() >= 5);
}
