//! L-Town (`BattLeDIM` 2020, data/README.md D2): what every real-data test on
//! it shares, so they read, watch and score the same way - the files'
//! clock, the holiday calendar, the true leaks, the night watch over the
//! whole town, and the one-to-one scoring (FN-051).

use jk_balance::{
    Direction, LeakAlarm, Meter, Night, NightWindow, balance, mark_holidays, night_flows,
};
use jk_files::scada::{ScadaOptions, ScadaTable, ScadaUnit, read};
use jk_kernel::Time;
use jk_kernel::clock::days_from_civil;

/// Seconds in a day.
pub const DAY: f64 = 86_400.0;
/// m^3/s in one m^3/h.
pub const M3H: f64 = 1.0 / 3600.0;
/// Tank T1's floor area [m^2]: 16 m across (L-TOWN.inp [TANKS]).
pub const TANK_AREA: f64 = std::f64::consts::PI * 8.0 * 8.0;

/// Cyprus's public holidays 2018-2019 (L-Town is laid out on a Cypriot
/// town): New Year, Epiphany, Green Monday, 25 March, 1 April, Orthodox
/// Good Friday, Easter Sunday and Monday, 1 May, Pentecost Monday
/// (Kataklysmos), 15 August, 1 October, 28 October, 24-26 December.
/// Checked 2026-10-06 against officeholidays.com (2019) and
/// allaboutlimassol.com (2018).
pub const CYPRUS_HOLIDAYS: [(i64, u32, u32); 32] = [
    (2018, 1, 1),
    (2018, 1, 6),
    (2018, 2, 19),
    (2018, 3, 25),
    (2018, 4, 1),
    (2018, 4, 6),
    (2018, 4, 8),
    (2018, 4, 9),
    (2018, 5, 1),
    (2018, 5, 28),
    (2018, 8, 15),
    (2018, 10, 1),
    (2018, 10, 28),
    (2018, 12, 24),
    (2018, 12, 25),
    (2018, 12, 26),
    (2019, 1, 1),
    (2019, 1, 6),
    (2019, 3, 11),
    (2019, 3, 25),
    (2019, 4, 1),
    (2019, 4, 26),
    (2019, 4, 28),
    (2019, 4, 29),
    (2019, 5, 1),
    (2019, 6, 17),
    (2019, 8, 15),
    (2019, 10, 1),
    (2019, 10, 28),
    (2019, 12, 24),
    (2019, 12, 25),
    (2019, 12, 26),
];

/// Seconds since 2018-01-01 00:00 of "YYYY-MM-DD hh:mm[:ss]".
///
/// # Panics
/// A stamp that is not of that form (a test bug).
#[must_use]
pub fn seconds(stamp: &str) -> f64 {
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

/// 2018-01-01 00:00 in seconds since 1970 (the files' clock, read as UTC).
#[must_use]
pub fn seconds_to_epoch() -> f64 {
    #[allow(clippy::cast_precision_loss)] // exact
    let d = days_from_civil(2018, 1, 1) as f64;
    d * DAY
}

/// The date of a time in seconds from 2018-01-01.
#[must_use]
pub fn date(t: f64) -> String {
    jk_kernel::clock::stamp(t + seconds_to_epoch(), None)[..10].to_string()
}

/// Reads a L-Town SCADA file (its files name no units; its README does).
///
/// # Panics
/// A file that cannot be read (a test bug, or a changed dataset).
#[must_use]
pub fn table(path: &std::path::Path, unit: ScadaUnit) -> ScadaTable {
    let options = ScadaOptions {
        unit: Some(unit),
        ..ScadaOptions::default()
    };
    read(path, &options).unwrap()
}

/// One column of two consecutive years, end to end (checked to join).
///
/// # Panics
/// A missing column, or years that do not follow on.
#[must_use]
pub fn both_years(years: &[ScadaTable; 2], name: &str) -> Vec<f64> {
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

/// The whole town as one zone (inflows p227 + p235, minus what tank T1
/// stores), night by night, holidays marked.
///
/// # Panics
/// Inputs that do not make a balance (a test bug).
#[must_use]
pub fn town_nights(flows: &[ScadaTable; 2], levels: &[ScadaTable; 2]) -> Vec<Night> {
    let meters = [
        Meter {
            name: "p227 (from R1)".into(),
            direction: Direction::In,
            flows: both_years(flows, "p227"),
        },
        Meter {
            name: "p235 (from R2)".into(),
            direction: Direction::In,
            flows: both_years(flows, "p235"),
        },
    ];
    let volume: Vec<f64> = both_years(levels, "T1")
        .iter()
        .map(|h| h * TANK_AREA)
        .collect();
    let used = balance(flows[0].start, flows[0].interval, &meters, Some(&volume)).unwrap();
    let window = NightWindow {
        utc_offset: Time::ZERO, // the readings are in the town's clock time
        ..NightWindow::default()
    };
    let mut nights = night_flows(&used, window).unwrap();
    let holidays: Vec<i64> = CYPRUS_HOLIDAYS
        .iter()
        .map(|&(y, m, d)| days_from_civil(y, m, d))
        .collect();
    mark_holidays(&mut nights, &holidays);
    nights
}

/// One true leak (times in seconds from 2018-01-01).
#[derive(Debug, Clone)]
pub struct TrueLeak {
    /// The leaking pipe's ID.
    pub pipe: String,
    /// When it began.
    pub start: f64,
    /// When it was repaired (or the data ended).
    pub end: f64,
    /// Sudden (true) or growing.
    pub abrupt: bool,
    /// When a growing leak stops growing (the published peakTime).
    pub peak_at: f64,
    /// Largest flow it reached [m^3/h].
    pub peak: f64,
}

/// The true leaks of `dataset_configuration.yaml`, their peak flows from
/// the leak-flow files of both years.
///
/// # Panics
/// A leak line that is not of the published form.
#[must_use]
pub fn true_leaks(yaml: &str, leak_flows: &[ScadaTable; 2]) -> Vec<TrueLeak> {
    yaml.lines()
        .map(str::trim)
        .filter(|l| l.starts_with("- p") && l.contains(',')) // not the meters ("- p227")
        .map(|l| {
            let f: Vec<&str> = l[2..].split(',').map(str::trim).collect();
            let pipe = f[0].to_string();
            let peak = leak_flows
                .iter()
                .filter_map(|t| t.column(&pipe))
                .flat_map(|c| c.values.iter().copied())
                .fold(0.0, f64::max)
                / M3H;
            TrueLeak {
                pipe,
                start: seconds(f[1]),
                end: seconds(f[2]),
                abrupt: f[4] == "abrupt",
                peak_at: seconds(f[5]),
                peak,
            }
        })
        .collect()
}

/// Scoring, one to one, as leak-detection benchmarks do (FN-051; `BattLeDIM`'s
/// detection score [VERIFY the exact rule]): in time order, each alarm is
/// matched to a leak RUNNING when it is raised (begun, not yet repaired) and
/// not matched before - the newest first (a rise is most likely the latest
/// leak); each leak to one alarm at most. Returns, per alarm, the index of
/// its leak (None: a false alarm). `e0`: 2018-01-01 in the alarms' clock.
#[must_use]
pub fn match_alarms(alarms: &[LeakAlarm], truth: &[TrueLeak], e0: f64) -> Vec<Option<usize>> {
    let mut order: Vec<usize> = (0..alarms.len()).collect();
    order.sort_by(|&x, &y| {
        alarms[x]
            .raised
            .value()
            .total_cmp(&alarms[y].raised.value())
    });
    let mut taken = vec![false; truth.len()];
    let mut matched = vec![None; alarms.len()];
    for k in order {
        let raised = alarms[k].raised.value() - e0;
        let pick = (0..truth.len())
            .filter(|&i| !taken[i] && truth[i].start <= raised && truth[i].end >= raised)
            .max_by(|&x, &y| truth[x].start.total_cmp(&truth[y].start));
        if let Some(i) = pick {
            taken[i] = true;
            matched[k] = Some(i);
        }
    }
    matched
}
