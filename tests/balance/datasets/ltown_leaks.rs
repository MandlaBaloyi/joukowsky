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
//! night counts. The checks below lock in what that gave on 2026-10-05
//! (25 of 33 within a month, 3 false alarms, longest silence 102 days):
//! a change that makes it worse fails. Still missed: holidays look like
//! leaks (Orthodox Easter, Pentecost Monday) - a holiday calendar as
//! context is the next step.

use jk_balance::{Direction, LeakOptions, Meter, NightWindow, balance, night_flows, watch};
use jk_files::scada::{ScadaOptions, ScadaTable, ScadaUnit, read};
use jk_kernel::Time;
use jk_kernel::clock::days_from_civil;
use jk_tests::{check, dataset};

const DAY: f64 = 86_400.0;
/// Tank T1's floor area [m^2]: 16 m across (L-TOWN.inp [TANKS]).
const TANK_AREA: f64 = std::f64::consts::PI * 8.0 * 8.0;
const M3H: f64 = 1.0 / 3600.0;

/// Seconds since 2018-01-01 00:00 of "YYYY-MM-DD hh:mm[:ss]".
fn seconds(stamp: &str) -> f64 {
    let (date, time) = stamp.trim().split_once(' ').unwrap();
    let d: Vec<i64> = date.split('-').map(|x| x.parse().unwrap()).collect();
    let t: Vec<f64> = time.split(':').map(|x| x.parse().unwrap()).collect();
    let days = days_from_civil(
        d[0],
        u32::try_from(d[1]).unwrap(),
        u32::try_from(d[2]).unwrap(),
    ) - days_from_civil(2018, 1, 1);
    #[allow(clippy::cast_precision_loss)] // a few hundred days
    let day_seconds = days as f64 * DAY;
    day_seconds + t[0] * 3600.0 + t[1] * 60.0 + t.get(2).copied().unwrap_or(0.0)
}

/// One column of two consecutive years, end to end (checked to join).
fn both_years(years: &[ScadaTable; 2], name: &str) -> Vec<f64> {
    let first = years[0].column(name).unwrap();
    #[allow(clippy::cast_precision_loss)] // a year of readings
    let joins = years[0].start.value() + years[0].interval.value() * first.values.len() as f64;
    assert!(
        (joins - years[1].start.value()).abs() < 1.0,
        "the two years must follow on"
    );
    let mut v = first.values.clone();
    v.extend_from_slice(&years[1].column(name).unwrap().values);
    v
}

/// One true leak.
struct Leak {
    pipe: String,
    start: f64,
    end: f64,
    abrupt: bool,
    /// Largest flow it reached [m^3/h].
    peak: f64,
}

fn leaks(yaml: &str, flows: &[ScadaTable; 2]) -> Vec<Leak> {
    yaml.lines()
        .map(str::trim)
        .filter(|l| l.starts_with("- p") && l.contains(',')) // not the meters ("- p227")
        .map(|l| {
            let f: Vec<&str> = l[2..].split(',').map(str::trim).collect();
            let pipe = f[0].to_string();
            let peak = flows
                .iter()
                .filter_map(|t| t.column(&pipe))
                .flat_map(|c| c.values.iter().copied())
                .fold(0.0, f64::max)
                / M3H;
            Leak {
                pipe,
                start: seconds(f[1]),
                end: seconds(f[2]),
                abrupt: f[4] == "abrupt",
                peak,
            }
        })
        .collect()
}

fn date(t: f64) -> String {
    jk_kernel::clock::stamp(t + seconds_to_epoch(), None)[..10].to_string()
}

/// 2018-01-01 00:00 in seconds since 1970 (for writing dates).
fn seconds_to_epoch() -> f64 {
    #[allow(clippy::cast_precision_loss)] // exact
    let d = days_from_civil(2018, 1, 1) as f64;
    d * DAY
}

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
    // L-Town's files name no units; its README gives them.
    let unit = |u| ScadaOptions {
        unit: Some(u),
        ..ScadaOptions::default()
    };
    let table = |k: usize, u| read(&paths[k], &unit(u)).unwrap();
    let flows = [
        table(0, ScadaUnit::CubicMetresPerHour),
        table(1, ScadaUnit::CubicMetresPerHour),
    ];
    let levels = [table(2, ScadaUnit::Metres), table(3, ScadaUnit::Metres)];
    let leak_flows = [
        table(4, ScadaUnit::CubicMetresPerHour),
        table(5, ScadaUnit::CubicMetresPerHour),
    ];
    let truth = leaks(&std::fs::read_to_string(&paths[6]).unwrap(), &leak_flows);

    // The zone: two inflows, and the tank's volume, both years end to end.
    let meters = [
        Meter {
            name: "p227 (from R1)".into(),
            direction: Direction::In,
            flows: both_years(&flows, "p227"),
        },
        Meter {
            name: "p235 (from R2)".into(),
            direction: Direction::In,
            flows: both_years(&flows, "p235"),
        },
    ];
    let volume: Vec<f64> = both_years(&levels, "T1")
        .iter()
        .map(|h| h * TANK_AREA)
        .collect();
    // The files' times are the town's own clock, read as seconds since 1970
    // (so the watch knows each night's weekday); times below are counted
    // from 2018-01-01 00:00.
    let e0 = flows[0].start.value();
    check(
        "the readings start at 2018-01-01 00:00",
        (e0 - seconds_to_epoch()).abs() < 1.0,
    );
    let used = balance(flows[0].start, flows[0].interval, &meters, Some(&volume)).unwrap();
    let window = NightWindow {
        utc_offset: Time::ZERO, // the readings are in the town's clock time
        ..NightWindow::default()
    };
    let nights = night_flows(&used, window).unwrap();
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
    // An alarm catches every leak that began after the previous alarm (a
    // leak already running then was in the watch's new baseline) and was
    // still running when the rise began. Growing leaks start weeks before
    // they are big enough to see, so the start may be long before the rise.
    println!(
        "    #  rise from   raised on   size [m3/h]      true leak(s) begun since the last alarm, running at the rise"
    );
    let mut caught = vec![None; truth.len()];
    let mut false_alarms = 0;
    let mut previous = f64::NEG_INFINITY;
    for (k, a) in w.alarms.iter().enumerate() {
        let from = a.first_night.value() - e0;
        let raised = a.raised.value() - e0;
        let matched: Vec<usize> = (0..truth.len())
            .filter(|&i| {
                let l = &truth[i];
                l.start > previous && l.start <= raised && l.end >= from
            })
            .collect();
        previous = raised;
        for &i in &matched {
            caught[i].get_or_insert(raised - truth[i].start);
        }
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
        "no silence over 120 days while a big leak comes and goes",
        missed == 0 || longest <= 120.0 * DAY,
    );
    check("repairs are seen: 5 or more falls", w.falls.len() >= 5);
}
