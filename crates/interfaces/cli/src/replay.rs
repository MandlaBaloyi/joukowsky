//! `jk replay SITE`: run the live engine over a site's recordings.
//!
//! Reads the site file (`jk_files::site`) and everything it names - the
//! network, every logger's export - and replays the recordings through the
//! live engine as the site would have sent them. Prints a report: the
//! data, every alert (its latest revision), supply cuts, each logger's
//! fatigue loading, and every note the readers made.

use std::fmt::Write as _;
use std::path::PathBuf;

use jk_application::{CollectingSink, Level, replay};
use jk_classification::EventClass;
use jk_kernel::Time;
use jk_kernel::clock::stamp;

pub const USAGE: &str = "
    jk replay SITE [OPTIONS]  run the live engine over a site's recordings

  SITE is a site file: the network, the loggers (node, export file, range,
  clock), ratings, zone, altitude, power losses. See `jk replay --example`.

  OPTIONS:
    --chunk S             seconds of each logger sent at a time [1]
    --out FILE            also write the report to FILE
    --example             print an example site file
";

const EXAMPLE: &str = "# A joukowsky site file. Paths are relative to this file.
[site]
name     = Riverside estate
network  = riverside.inp        # EPANET network file
zone     = +02:00               # local time minus UTC (South Africa: +02:00)
altitude = 1450                 # m above sea level: where water boils
rating   = PN10                 # every pipe, unless listed under [ratings]
dates    = day-first            # 01/02/2026 is 1 February (or month-first)

[logger S1 gatehouse]           # the name used in alerts
node   = J-12                   # the network node it sits on
file   = s1.csv                 # its export
range  = 0 16 bar               # its measuring range
clock  = gnss                   # gnss (1 us), ntp (40 ms) or seconds

[logger S2 north block]
node   = J-40
file   = s2.csv
unit   = bar                    # for a column whose header names none

[ratings]
P-17 = PN16                     # or e.g. 160 m, 16 bar

[power]
monitored = yes                 # pump power is logged at this site
lost      = 2026-10-01 14:02:00 # each recorded loss (local time unless zoned)
";

/// What `jk replay` was asked for.
pub struct Args {
    site: PathBuf,
    chunk: f64,
    out: Option<PathBuf>,
}

/// Reads the options after `replay`; `Ok(None)` when only the example was
/// asked for (printed).
///
/// # Errors
/// No site file, an unknown option or a bad value.
pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Option<Args>, String> {
    let mut site = None;
    let mut chunk = 1.0;
    let mut out = None;
    while let Some(a) = it.next() {
        match a.as_str() {
            "--example" => {
                print!("{EXAMPLE}");
                return Ok(None);
            }
            "--chunk" => {
                let v = it.next().ok_or("--chunk needs seconds")?;
                chunk = v
                    .parse::<f64>()
                    .ok()
                    .filter(|c| *c > 0.0)
                    .ok_or_else(|| format!("--chunk: '{v}' is not a positive number"))?;
            }
            "--out" => out = Some(PathBuf::from(it.next().ok_or("--out needs a file name")?)),
            s if s.starts_with("--") => return Err(format!("unknown option {s}")),
            s => site = Some(PathBuf::from(s)),
        }
    }
    let site = site.ok_or("replay needs a site file (see `jk replay --example`)")?;
    Ok(Some(Args { site, chunk, out }))
}

/// Replays the site and prints (and writes) the report.
///
/// # Errors
/// A site that cannot be read or replayed, or a report that cannot be
/// written.
pub fn run(a: &Args) -> Result<(), String> {
    eprintln!("jk replay: reading {}...", a.site.display());
    let loaded = jk_files::site::load(&a.site).map_err(|e| format!("{}: {e}", a.site.display()))?;
    eprintln!("jk replay: replaying through the live engine...");
    let mut sink = CollectingSink::default();
    let result = replay(
        &loaded.site,
        &loaded.config,
        &loaded.records,
        Time::new(a.chunk),
        &mut sink,
    )
    .map_err(|e| e.to_string())?;
    let report = report(&loaded, &result);
    print!("{report}");
    if let Some(out) = &a.out {
        std::fs::write(out, &report).map_err(|e| format!("cannot write {}: {e}", out.display()))?;
        eprintln!("jk replay: report written to {}", out.display());
    }
    Ok(())
}

/// The report, as text.
pub fn report(loaded: &jk_files::site::LoadedSite, r: &jk_application::Replay) -> String {
    let site = &loaded.site;
    let zone = site.zone();
    let mut s = String::new();
    let _ = writeln!(s, "joukowsky replay - {}", loaded.name);
    let _ = writeln!(
        s,
        "  network: {} nodes, {} pipes; {} logger(s)",
        site.network.nodes().len(),
        site.network.pipes().len(),
        site.sensors.len()
    );
    let _ = writeln!(
        s,
        "  data: {} to {} ({:.1} min)",
        stamp(r.span.0, zone),
        stamp(r.span.1, zone),
        (r.span.1 - r.span.0) / 60.0
    );
    for f in &loaded.records {
        let samples: usize = f.runs.iter().map(|(_, h)| h.len()).sum();
        let _ = writeln!(
            s,
            "    {}: {samples} samples at {:.0} S/s in {} run(s){}",
            site.sensors[f.sensor].name,
            1.0 / f.interval.value(),
            f.runs.len(),
            if f.runs.len() > 1 {
                " - gaps between them, never filled"
            } else {
                ""
            }
        );
    }
    let latest = r.latest();
    let count = |l: Level| latest.iter().filter(|a| a.level == l).count();
    let _ = writeln!(
        s,
        "\nALERTS: {} event(s) - {} alarm, {} warning, {} review, {} info ({} messages with revisions)",
        latest.len(),
        count(Level::Alarm),
        count(Level::Warning),
        count(Level::Review),
        count(Level::Info),
        r.alerts.len()
    );
    for a in &latest {
        let _ = writeln!(s, "{}", a.render(&site.pipe_names));
    }
    let cuts = latest
        .iter()
        .filter(|a| a.class == EventClass::SupplyCut)
        .count();
    let _ = writeln!(s, "\nSUPPLY CUTS: {cuts}");
    let _ = writeln!(
        s,
        "\nFATIGUE LOADING - equivalent swings of 50 m (S-N exponent 4 [VERIFY per material]):"
    );
    let mut ranked: Vec<(String, f64)> = site
        .sensors
        .iter()
        .zip(&r.loading)
        .map(|(sensor, l)| (sensor.name.clone(), l.equivalent_cycles(50.0, 4.0)))
        .collect();
    ranked.sort_by(|a, b| b.1.total_cmp(&a.1));
    for (name, n) in ranked {
        let _ = writeln!(s, "  {name:<24} {n:10.2}");
    }
    let _ = writeln!(
        s,
        "\nNOTES - what the readers assumed or repaired ({}):",
        loaded.notes.len()
    );
    for n in &loaded.notes {
        let _ = writeln!(s, "  - {n}");
    }
    s
}
