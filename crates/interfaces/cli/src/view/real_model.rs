//! The REAL data (data/README.md) run through the engine, for the "Real
//! data" pages (`jk view --real`): what the engine said next to what
//! really happened.
//!
//! - [`LabModel`]: the Texas A&M testbed's 20 valve closures, averaged to
//!   128 readings a second as a logger would, through the detector
//!   (FN-047; `tests/detection/datasets/tamu_valve.rs`).
//! - [`TownWeek`]: L-Town's first week played by the engine from its model
//!   alone, next to the published readings - pump, tank, pressures (FN-045;
//!   `tests/adapters/datasets/ltown.rs`).
//! - [`LeakModel`]: L-Town's two years of meter readings through the night
//!   watch, against the 33 true leaks (FN-048;
//!   `tests/balance/datasets/ltown_leaks.rs`).
//!
//! Each part is built only if its files are on this machine; otherwise the
//! page says which file is missing. Nothing is downloaded.

use std::path::{Path, PathBuf};

use jk_balance::{
    Direction, LeakOptions, Meter, NightWindow, Watch, balance, mark_holidays, night_flows, watch,
};
use jk_detection::{Detector, DetectorConfig, Event, Series};
use jk_files::epanet::InpNetwork;
use jk_files::logger::{CsvOptions, PressureUnit};
use jk_files::scada::{ScadaOptions, ScadaTable, ScadaUnit};
use jk_hydraulics::steady::controls::Target;
use jk_hydraulics::steady::extended::SupplyEvent;
use jk_hydraulics::{Node, NodeId, SteadyOptions};
use jk_kernel::Flow;
use jk_kernel::Time;
use jk_kernel::clock::days_from_civil;
use jk_localisation::{Signatures, locate_from_pressures};

const DAY: f64 = 86_400.0;
const M3H: f64 = 1.0 / 3600.0;
/// Lab readings averaged into one logger sample (25 600 / 128).
const BLOCK: usize = 200;
/// The lab runs: layout and leak (BR branched, LO looped; NL no leak, OL
/// orifice, CC/LC circumferential/longitudinal crack, GL gasket).
pub const LAB_RUNS: [&str; 10] = [
    "BR_NL", "BR_OL", "BR_CC", "BR_LC", "BR_GL", "LO_NL", "LO_OL", "LO_CC", "LO_LC", "LO_GL",
];
/// L-Town's tank T1: 16 m across (L-TOWN.inp [TANKS]), floor 98.68 m.
const TANK_AREA: f64 = std::f64::consts::PI * 8.0 * 8.0;
/// Cyprus's public holidays 2018-2019, L-Town's calendar for the night
/// watch (the same list as `tests/balance/datasets/ltown_leaks.rs`, with
/// its sources).
const CYPRUS_HOLIDAYS: [(i64, u32, u32); 32] = [
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
/// Pressure sensors shown on the town page (of the 33).
const SHOWN_SENSORS: [&str; 3] = ["n1", "n105", "n288"];

/// One lab recording as a 128 S/s logger would have it, and what the
/// detector saw.
pub struct LabTrace {
    /// Run and sensor, e.g. `BR_NL P2`.
    pub name: String,
    /// Heads [m] (pressure change: these sensors do not hold the static
    /// pressure), every `interval` from 0.
    pub heads: Vec<f64>,
    pub interval: f64,
    pub events: Vec<Event>,
    /// Whether this trace saw the closure (a rise of 0.2 m or more at
    /// 16-23 s).
    pub closure: bool,
}

pub struct LabModel {
    pub traces: Vec<LabTrace>,
    /// Fronts before 16 s (nothing happens then): run, sensor, time.
    pub early: Vec<String>,
}

/// One pump switch: the engine's time, the published time [h], on/off.
pub struct Switch {
    pub engine: f64,
    pub published: f64,
    pub on: bool,
}

/// How close the engine's pressures are to the published ones over the
/// week, at every pressure sensor.
pub struct PressureFit {
    pub sensors: usize,
    /// Mean of each sensor's mean absolute difference [m].
    pub mean: f64,
    /// The largest of those [m], and where.
    pub worst: f64,
    pub worst_at: String,
}

pub struct TownWeek {
    pub switches: Vec<Switch>,
    pub pressure_fit: PressureFit,
    /// Tank level [m] every `interval` s: engine, published.
    pub tank: (Vec<f64>, Vec<f64>),
    /// Pressure [m] at a few sensors: name, engine, published.
    pub pressures: Vec<(String, Vec<f64>, Vec<f64>)>,
    pub interval: f64,
}

/// One true leak.
pub struct TrueLeak {
    pub pipe: String,
    /// Days from 2018-01-01.
    pub start: f64,
    pub end: f64,
    pub abrupt: bool,
    /// Largest flow [m3/h].
    pub peak: f64,
    /// Days from its start to the alarm that caught it.
    pub caught_after: Option<f64>,
}

pub struct LeakModel {
    /// Night flows [m3/h], one per night from 2018-01-01.
    pub nights: Vec<f64>,
    pub watch: Watch,
    pub truth: Vec<TrueLeak>,
    /// For each alarm: the leaks it caught (indices into `truth`).
    pub alarm_leaks: Vec<Vec<usize>>,
    /// Seconds since 1970 of 2018-01-01 00:00.
    pub e0: f64,
}

impl LeakModel {
    /// Days from 2018-01-01 of a watch time.
    pub fn day(&self, t: Time) -> f64 {
        (t.value() - self.e0) / DAY
    }
}

/// All the real data found.
pub struct RealModel {
    pub lab: Result<LabModel, String>,
    pub town: Result<TownWeek, String>,
    pub leaks: Result<LeakModel, String>,
    /// Where each leak alarm points, from the pressure sensors.
    pub places: Result<LeakPlaces, String>,
}

/// Where the engine pointed for one alarm, from the pressure sensors.
pub struct Place {
    /// The best junction (index into the network's nodes).
    pub node: usize,
    /// The leak's flow the fit implies [m3/h].
    pub flow: f64,
    /// Distance to the leak the alarm was matched to [m].
    pub to_matched: f64,
    /// Distance to the nearest leak whose night flow truly changed by
    /// 3 m3/h or more between the two weeks [m]; None: none did.
    pub to_changed: Option<f64>,
}

/// The town drawn, its pressure sensors, and a place per alarm (None: the
/// alarm matched no leak, or too near the data's ends).
pub struct LeakPlaces {
    pub inp: InpNetwork,
    /// Pressure sensors (node indices).
    pub sensors: Vec<usize>,
    pub places: Vec<Option<Place>>,
    /// Seconds to make the 782 signatures.
    pub seconds: f64,
}

impl RealModel {
    /// Runs every dataset found under `data` (the project's `data/`).
    pub fn run(data: &Path) -> Self {
        let leaks = leaks(data);
        let places = match &leaks {
            Ok(l) => places(data, l),
            Err(e) => Err(e.clone()),
        };
        Self {
            lab: lab(data),
            town: town(data),
            leaks,
            places,
        }
    }
}

fn file(data: &Path, rel: &str) -> Result<PathBuf, String> {
    let p = data.join(rel);
    if p.exists() {
        Ok(p)
    } else {
        Err(format!(
            "{} is not on this machine (see data/README.md)",
            p.display()
        ))
    }
}

fn lab(data: &Path) -> Result<LabModel, String> {
    let options = CsvOptions {
        unit: Some(PressureUnit::Pascal),
        ..CsvOptions::default()
    };
    let detector = Detector::new(DetectorConfig::default()).map_err(|e| e.to_string())?;
    let mut traces = Vec::new();
    let mut early = Vec::new();
    for run in LAB_RUNS {
        for sensor in ["P1", "P2"] {
            let path = file(
                data,
                &format!("datasets/tamu-testbed/{run}_Transient_{sensor}.csv"),
            )?;
            let rec = jk_files::logger::read(&path, &options).map_err(|e| e.to_string())?;
            let raw = &rec.runs[0].heads[0];
            let interval = rec.interval.value() * to_f64(BLOCK);
            let heads: Vec<f64> = raw
                .as_chunks::<BLOCK>()
                .0
                .iter()
                .map(|b| b.iter().sum::<f64>() / to_f64(BLOCK))
                .collect();
            let series = Series {
                samples: &heads,
                start: Time::new(interval),
                interval: Time::new(interval),
            };
            let events = detector.run(&series).map_err(|e| e.to_string())?.events;
            let closure = events
                .iter()
                .any(|e| e.step.value() >= 0.2 && (16.0..23.0).contains(&e.arrival.value()));
            early.extend(
                events
                    .iter()
                    .filter(|e| e.arrival.value() < 16.0)
                    .map(|e| format!("{run} {sensor} at {:.2} s", e.arrival.value())),
            );
            traces.push(LabTrace {
                name: format!("{run} {sensor}"),
                heads,
                interval,
                events,
                closure,
            });
        }
    }
    Ok(LabModel { traces, early })
}

fn scada(data: &Path, rel: &str, unit: ScadaUnit) -> Result<ScadaTable, String> {
    let options = ScadaOptions {
        unit: Some(unit),
        ..ScadaOptions::default()
    };
    jk_files::scada::read(&file(data, rel)?, &options).map_err(|e| e.to_string())
}

fn column<'a>(t: &'a ScadaTable, name: &str) -> Result<&'a [f64], String> {
    t.column(name)
        .map(|c| c.values.as_slice())
        .ok_or_else(|| format!("no column {name}"))
}

#[allow(clippy::too_many_lines)] // read, run, then compare - one story
fn town(data: &Path) -> Result<TownWeek, String> {
    let inp = jk_files::epanet::read(
        &file(data, "incoming/ltown/L-TOWN.inp")?,
        &jk_files::epanet::InpOptions::default(),
    )
    .map_err(|e| e.to_string())?;
    let flows = scada(
        data,
        "incoming/ltown/2018_SCADA_Flows.csv",
        ScadaUnit::CubicMetresPerHour,
    )?;
    let levels = scada(
        data,
        "incoming/ltown/2018_SCADA_Levels.csv",
        ScadaUnit::Metres,
    )?;
    let pressures = scada(
        data,
        "incoming/ltown/2018_SCADA_Pressures.csv",
        ScadaUnit::Metres,
    )?;
    let period = &inp.operations.period;
    let run = inp.network.extended(period).map_err(|e| e.to_string())?;
    let interval = flows.interval.value();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a week of slots
    let slots = (period.duration.value() / interval) as usize;

    // The engine's state at each slot: the last recorded state at or before.
    let times: Vec<f64> = run.times.iter().map(|t| t.value()).collect();
    let at = |k: usize| -> usize {
        let t = to_f64(k) * interval;
        times.partition_point(|&x| x <= t + 1e-6).saturating_sub(1)
    };
    let node = |id: &str| inp.node_ids.iter().position(|n| n == id);
    let tank_node = node("T1").ok_or("no tank T1")?;
    let tank_floor = inp.elevations[tank_node].map_or(0.0, jk_kernel::Length::value);
    let tank_engine: Vec<f64> = (0..slots)
        .map(|k| run.node_heads[at(k)][tank_node].value() - tank_floor)
        .collect();
    let tank_published = column(&levels, "T1")?[..slots].to_vec();
    let mut fits = Vec::new();
    for c in &pressures.columns {
        let Some(n) = node(&c.name) else { continue };
        let ground = inp.elevations[n].map_or(0.0, jk_kernel::Length::value);
        let diffs: Vec<f64> = (0..slots.min(c.values.len()))
            .map(|k| (run.node_heads[at(k)][n].value() - ground - c.values[k]).abs())
            .filter(|d| d.is_finite())
            .collect();
        if !diffs.is_empty() {
            fits.push((
                c.name.clone(),
                diffs.iter().sum::<f64>() / to_f64(diffs.len()),
            ));
        }
    }
    let (worst_at, worst) = fits
        .iter()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .cloned()
        .unwrap_or_default();
    let pressure_fit = PressureFit {
        sensors: fits.len(),
        mean: fits.iter().map(|f| f.1).sum::<f64>() / to_f64(fits.len().max(1)),
        worst,
        worst_at,
    };
    let mut shown = Vec::new();
    for id in SHOWN_SENSORS {
        let n = node(id).ok_or_else(|| format!("no node {id}"))?;
        let ground = inp.elevations[n].map_or(0.0, jk_kernel::Length::value);
        let engine: Vec<f64> = (0..slots)
            .map(|k| run.node_heads[at(k)][n].value() - ground)
            .collect();
        shown.push((
            id.to_string(),
            engine,
            column(&pressures, id)?[..slots].to_vec(),
        ));
    }

    // Pump switches: the engine's events, and the published pump flow.
    let engine: Vec<(f64, bool)> = run
        .events
        .iter()
        .filter_map(|e| match e {
            SupplyEvent::Switched {
                target: Target::Pump(_),
                open,
                at,
                ..
            } => Some((at.value(), *open)),
            _ => None,
        })
        .collect();
    let pump = column(&flows, "PUMP_1")?;
    let mut published = Vec::new();
    let mut was: Option<bool> = None;
    for (k, &q) in pump.iter().enumerate().take(slots) {
        let running = q / M3H > 1.0;
        if was.is_some_and(|w| w != running) {
            published.push((to_f64(k) * interval - interval / 2.0, running));
        }
        was = Some(running);
    }
    let switches = engine
        .iter()
        .zip(&published)
        .map(|(e, p)| Switch {
            engine: e.0 / 3600.0,
            published: p.0 / 3600.0,
            on: p.1,
        })
        .collect();
    Ok(TownWeek {
        switches,
        pressure_fit,
        tank: (tank_engine, tank_published),
        pressures: shown,
        interval,
    })
}

/// Days since 2018-01-01 of "YYYY-MM-DD hh:mm".
fn days(stamp: &str) -> f64 {
    let (date, time) = stamp.trim().split_once(' ').unwrap_or((stamp, "00:00"));
    let d: Vec<i64> = date.split('-').filter_map(|x| x.parse().ok()).collect();
    let t: Vec<f64> = time.split(':').filter_map(|x| x.parse().ok()).collect();
    if d.len() < 3 || t.len() < 2 {
        return f64::NAN;
    }
    let (Ok(m), Ok(dd)) = (u32::try_from(d[1]), u32::try_from(d[2])) else {
        return f64::NAN;
    };
    #[allow(clippy::cast_precision_loss)] // a few hundred days
    let whole = (days_from_civil(d[0], m, dd) - days_from_civil(2018, 1, 1)) as f64;
    whole + (t[0] * 3600.0 + t[1] * 60.0) / DAY
}

#[allow(clippy::too_many_lines)] // read, watch, then score - one story
fn leaks(data: &Path) -> Result<LeakModel, String> {
    let u = ScadaUnit::CubicMetresPerHour;
    let flows = [
        scada(data, "incoming/ltown/2018_SCADA_Flows.csv", u)?,
        scada(data, "incoming/ltown/2019_SCADA_Flows.csv", u)?,
    ];
    let levels = [
        scada(
            data,
            "incoming/ltown/2018_SCADA_Levels.csv",
            ScadaUnit::Metres,
        )?,
        scada(
            data,
            "incoming/ltown/2019_SCADA_Levels.csv",
            ScadaUnit::Metres,
        )?,
    ];
    let leak_flows = [
        scada(data, "incoming/ltown/2018_Leakages.csv", u)?,
        scada(data, "incoming/ltown/2019_Leakages.csv", u)?,
    ];
    let yaml = std::fs::read_to_string(file(data, "incoming/ltown/dataset_configuration.yaml")?)
        .map_err(|e| e.to_string())?;
    let both = |t: &[ScadaTable; 2], name: &str| -> Result<Vec<f64>, String> {
        let mut v = column(&t[0], name)?.to_vec();
        v.extend_from_slice(column(&t[1], name)?);
        Ok(v)
    };
    let meters = [
        Meter {
            name: "p227".into(),
            direction: Direction::In,
            flows: both(&flows, "p227")?,
        },
        Meter {
            name: "p235".into(),
            direction: Direction::In,
            flows: both(&flows, "p235")?,
        },
    ];
    let volume: Vec<f64> = both(&levels, "T1")?.iter().map(|h| h * TANK_AREA).collect();
    let used = balance(flows[0].start, flows[0].interval, &meters, Some(&volume))
        .map_err(|e| e.to_string())?;
    let window = NightWindow {
        utc_offset: Time::ZERO,
        ..NightWindow::default()
    };
    let mut nights = night_flows(&used, window).map_err(|e| e.to_string())?;
    let holidays: Vec<i64> = CYPRUS_HOLIDAYS
        .iter()
        .map(|&(y, m, d)| days_from_civil(y, m, d))
        .collect();
    mark_holidays(&mut nights, &holidays);
    let w = watch(&nights, &LeakOptions::default()).map_err(|e| e.to_string())?;
    let e0 = flows[0].start.value();

    let mut truth: Vec<TrueLeak> = yaml
        .lines()
        .map(str::trim)
        .filter(|l| l.starts_with("- p") && l.contains(','))
        .filter_map(|l| {
            let f: Vec<&str> = l[2..].split(',').map(str::trim).collect();
            (f.len() >= 5).then(|| {
                let pipe = f[0].to_string();
                let peak = leak_flows
                    .iter()
                    .filter_map(|t| t.column(&pipe))
                    .flat_map(|c| c.values.iter().copied())
                    .fold(0.0, f64::max)
                    / M3H;
                TrueLeak {
                    pipe,
                    start: days(f[1]),
                    end: days(f[2]),
                    abrupt: f[4] == "abrupt",
                    peak,
                    caught_after: None,
                }
            })
        })
        .collect();
    // One-to-one, newest first (the scoring of the tests, FN-051): in time
    // order each alarm takes the newest leak running when it is raised and
    // not taken before.
    let mut alarm_leaks = vec![Vec::new(); w.alarms.len()];
    let mut order: Vec<usize> = (0..w.alarms.len()).collect();
    order.sort_by(|&x, &y| {
        w.alarms[x]
            .raised
            .value()
            .total_cmp(&w.alarms[y].raised.value())
    });
    for k in order {
        let raised = (w.alarms[k].raised.value() - e0) / DAY;
        let pick = (0..truth.len())
            .filter(|&i| {
                truth[i].caught_after.is_none()
                    && truth[i].start <= raised
                    && truth[i].end >= raised
            })
            .max_by(|&x, &y| truth[x].start.total_cmp(&truth[y].start));
        if let Some(i) = pick {
            truth[i].caught_after = Some(raised - truth[i].start);
            alarm_leaks[k].push(i);
        }
    }
    Ok(LeakModel {
        nights: nights.iter().map(|n| n.flow / M3H).collect(),
        watch: w,
        truth,
        alarm_leaks,
        e0,
    })
}

fn to_f64(n: usize) -> f64 {
    #[allow(clippy::cast_precision_loss)] // small counts
    let x = n as f64;
    x
}

/// Mean of `v` (0 for none).
fn mean(v: &[f64]) -> f64 {
    v.iter().sum::<f64>() / to_f64(v.len().max(1))
}

/// Where each leak alarm points, from the 33 pressure sensors and the
/// town's model (localisation/src/pressure.rs, FN-053; the same windows
/// and scoring as `tests/localisation/datasets/ltown_pressure.rs`): the
/// sensors' night pressures in the week the rise began minus the week
/// before, fitted to a 10 m3/h leak's signature at every junction.
#[allow(clippy::too_many_lines)] // read, make signatures, locate each alarm - one story
fn places(data: &Path, l: &LeakModel) -> Result<LeakPlaces, String> {
    let inp = jk_files::epanet::read(
        &file(data, "incoming/ltown/L-TOWN.inp")?,
        &jk_files::epanet::InpOptions::default(),
    )
    .map_err(|e| e.to_string())?;
    let u = ScadaUnit::Metres;
    let tables = [
        scada(data, "incoming/ltown/2018_SCADA_Pressures.csv", u)?,
        scada(data, "incoming/ltown/2019_SCADA_Pressures.csv", u)?,
    ];
    let leak_tables = [
        scada(
            data,
            "incoming/ltown/2018_Leakages.csv",
            ScadaUnit::CubicMetresPerHour,
        )?,
        scada(
            data,
            "incoming/ltown/2019_Leakages.csv",
            ScadaUnit::CubicMetresPerHour,
        )?,
    ];
    let sensor_ids: Vec<String> = tables[0].columns.iter().map(|c| c.name.clone()).collect();
    let index = |id: &str| {
        inp.node_ids
            .iter()
            .position(|n| n == id)
            .ok_or_else(|| format!("no node {id}"))
    };
    let sensors: Vec<usize> = sensor_ids
        .iter()
        .map(|s| index(s))
        .collect::<Result<_, _>>()?;
    let id = |k: usize| inp.network.node_id(k).ok_or("bad node");

    // The town at 03:00, a 10 m3/h leak's signature at every junction.
    let mut net = inp.network.clone();
    let m = inp
        .operations
        .period
        .multipliers_at(net.nodes().len(), Time::new(3.0 * 3600.0));
    let mut candidates = Vec::new();
    for (k, &mk) in m.iter().enumerate() {
        if let Node::Junction { demand } = net.nodes()[k] {
            net.set_demand(id(k)?, demand * mk)
                .map_err(|e| e.to_string())?;
            if !inp.node_ids[k].contains('/') && !inp.node_ids[k].contains('=') {
                candidates.push(id(k)?);
            }
        }
    }
    let sensor_nodes: Vec<NodeId> = sensors.iter().map(|&k| id(k)).collect::<Result<_, _>>()?;
    let started = std::time::Instant::now();
    let signatures = Signatures::compute(
        &net,
        &sensor_nodes,
        &candidates,
        Flow::new(10.0 * M3H),
        &SteadyOptions::default(),
    )
    .map_err(|e| e.to_string())?;
    let seconds = started.elapsed().as_secs_f64();

    // Night (02:00-04:00) means: pressures per sensor, flows per true leak.
    let night = |values: &[f64], d: usize| mean(&values[d * 288 + 24..d * 288 + 48]);
    let mut pressures: Vec<Vec<f64>> = Vec::new();
    for t in &tables {
        let n = t.columns[0].values.len() / 288;
        for d in 0..n {
            pressures.push(
                sensor_ids
                    .iter()
                    .map(|s| t.column(s).map_or(f64::NAN, |c| night(&c.values, d)))
                    .collect(),
            );
        }
    }
    let leak_nights: Vec<Vec<f64>> = l
        .truth
        .iter()
        .map(|t| {
            leak_tables
                .iter()
                .flat_map(|tab| {
                    let n = tab.columns[0].values.len() / 288;
                    (0..n)
                        .map(|d| {
                            tab.column(&t.pipe)
                                .map_or(0.0, |c| night(&c.values, d) / M3H)
                        })
                        .collect::<Vec<_>>()
                })
                .collect()
        })
        .collect();
    // Each sensor's night-to-night noise (same weekday a week apart), >= 2 cm.
    let noise: Vec<f64> = (0..sensors.len())
        .map(|s| {
            let mut d: Vec<f64> = (7..pressures.len())
                .map(|k| (pressures[k][s] - pressures[k - 7][s]).abs())
                .collect();
            d.sort_by(f64::total_cmp);
            (1.4826 * d[d.len() / 2] / std::f64::consts::SQRT_2).max(0.02)
        })
        .collect();
    let pos = |k: usize| inp.positions[k].unwrap_or((f64::NAN, f64::NAN));
    let pipe_at = |pipe: &str| -> (f64, f64) {
        inp.pipe_ids
            .iter()
            .position(|x| x == pipe)
            .map_or((f64::NAN, f64::NAN), |k| {
                let p = &inp.network.pipes()[k];
                let (a, b) = (pos(p.from.index()), pos(p.to.index()));
                (f64::midpoint(a.0, b.0), f64::midpoint(a.1, b.1))
            })
    };
    let n = pressures.len();
    let mut places = Vec::new();
    for (a, caught) in l.watch.alarms.iter().zip(&l.alarm_leaks) {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a day count
        let rose = l.day(a.first_night).floor().max(0.0) as usize;
        let Some(&i) = caught.first() else {
            places.push(None);
            continue;
        };
        if rose < 8 || rose + 8 > n {
            places.push(None);
            continue;
        }
        let before: Vec<f64> = (0..sensors.len())
            .map(|s| {
                mean(
                    &pressures[rose - 8..rose - 1]
                        .iter()
                        .map(|r| r[s])
                        .collect::<Vec<_>>(),
                )
            })
            .collect();
        let after: Vec<f64> = (0..sensors.len())
            .map(|s| {
                mean(
                    &pressures[rose..rose + 7]
                        .iter()
                        .map(|r| r[s])
                        .collect::<Vec<_>>(),
                )
            })
            .collect();
        let change: Vec<f64> = after.iter().zip(&before).map(|(x, y)| x - y).collect();
        let Some(best) = locate_from_pressures(&signatures, &change, &noise)
            .map_err(|e| e.to_string())?
            .first()
            .copied()
        else {
            places.push(None);
            continue;
        };
        let at = pos(best.node.index());
        let dist = |p: (f64, f64)| (at.0 - p.0).hypot(at.1 - p.1);
        let to_changed = (0..l.truth.len())
            .filter(|&k| {
                let f = &leak_nights[k];
                (mean(&f[rose..rose + 7]) - mean(&f[rose - 8..rose - 1])).abs() >= 3.0
            })
            .map(|k| dist(pipe_at(&l.truth[k].pipe)))
            .reduce(f64::min);
        places.push(Some(Place {
            node: best.node.index(),
            flow: best.flow.value() / M3H,
            to_matched: dist(pipe_at(&l.truth[i].pipe)),
            to_changed,
        }));
    }
    Ok(LeakPlaces {
        inp,
        sensors,
        places,
        seconds,
    })
}
