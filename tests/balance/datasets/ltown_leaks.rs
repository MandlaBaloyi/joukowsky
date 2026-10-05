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
//! Stage: FEED AND LOOK - it prints the score and judges nothing yet.
//! What it showed (FN-048) is being fixed; the checks come with the fix.

use jk_balance::{Direction, LeakOptions, Meter, NightWindow, balance, night_flows, watch};
use jk_kernel::Time;
use jk_kernel::clock::days_from_civil;
use jk_tests::dataset;

const READING: f64 = 300.0;
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

/// A semicolon file with decimal commas: the header and the rows (without
/// the timestamp column, which is checked to be every 5 minutes from
/// `from`).
fn table(text: &str, from: f64) -> (Vec<String>, Vec<Vec<f64>>) {
    let mut lines = text.lines();
    let header: Vec<String> = lines
        .next()
        .unwrap()
        .split(';')
        .skip(1)
        .map(str::to_string)
        .collect();
    let rows: Vec<Vec<f64>> = lines
        .enumerate()
        .map(|(k, line)| {
            let mut f = line.split(';');
            let t = seconds(f.next().unwrap());
            #[allow(clippy::cast_precision_loss)] // small
            let expected = from + k as f64 * READING;
            assert!(
                (t - expected).abs() < 1.0,
                "row {k}: a reading missing or out of order"
            );
            f.map(|v| v.replace(',', ".").parse().unwrap()).collect()
        })
        .collect();
    (header, rows)
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

fn leaks(yaml: &str, leak_tables: &[(Vec<String>, Vec<Vec<f64>>)]) -> Vec<Leak> {
    yaml.lines()
        .map(str::trim)
        .filter(|l| l.starts_with("- p") && l.contains(',')) // not the meters ("- p227")
        .map(|l| {
            let f: Vec<&str> = l[2..].split(',').map(str::trim).collect();
            let pipe = f[0].to_string();
            let peak = leak_tables
                .iter()
                .filter_map(|(h, rows)| {
                    let col = h.iter().position(|p| *p == pipe)?;
                    Some(rows.iter().map(|r| r[col]).fold(0.0, f64::max))
                })
                .fold(0.0, f64::max);
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
    let read = |k: usize| std::fs::read_to_string(&paths[k]).unwrap();
    let y2019 = seconds("2019-01-01 00:00");
    let flows: Vec<_> = [table(&read(0), 0.0), table(&read(1), y2019)].into();
    let levels: Vec<_> = [table(&read(2), 0.0), table(&read(3), y2019)].into();
    let truth = leaks(&read(6), &[table(&read(4), 0.0), table(&read(5), y2019)]);

    // The zone: two inflows, and the tank's volume, both years end to end.
    let column = |t: &[(Vec<String>, Vec<Vec<f64>>)], name: &str, scale: f64| -> Vec<f64> {
        t.iter()
            .flat_map(|(h, rows)| {
                let c = h.iter().position(|x| x == name).unwrap();
                rows.iter().map(move |r| r[c] * scale)
            })
            .collect()
    };
    let meters = [
        Meter {
            name: "p227 (from R1)".into(),
            direction: Direction::In,
            flows: column(&flows, "p227", M3H),
        },
        Meter {
            name: "p235 (from R2)".into(),
            direction: Direction::In,
            flows: column(&flows, "p235", M3H),
        },
    ];
    let volume = column(&levels, "T1", TANK_AREA);
    let used = balance(Time::ZERO, Time::new(READING), &meters, Some(&volume)).unwrap();
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
        let from = a.first_night.value();
        let raised = a.raised.value();
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
    marks.extend(w.alarms.iter().map(|a| a.raised.value()));
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
}
