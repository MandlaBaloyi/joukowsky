//! V-S11: a whole site through files - a site file, an EPANET network,
//! three logger exports - read by `jk_files::site::load` and replayed
//! through the live engine (`jk_application::replay`), as `jk replay`
//! does.
//!
//! The estate of `logger_files.rs`, a 10 mm hole opening 200 m along T-S3
//! at 3 s. The exports are written as a South African logger would: local
//! time (UTC+2) with NO zone in the stamps, pressure in bar; one export's
//! header names no unit (the site file gives it). The site file gives the
//! zone, altitude (1450 m), ratings, ranges, clocks and that pump power is
//! monitored. The engine must read local times as UTC+2, raise ONE alarm
//! on P4 near 200 m, and write its time on the local clock.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use jk_application::{CollectingSink, Level, replay};
use jk_hydraulics::boundary::leak;
use jk_hydraulics::{Burst, BurstOpening, GridOptions, InitialCondition, Probe};
use jk_kernel::{Area, Head, Length, STANDARD_GRAVITY, Time};
use jk_tests::estate::estate;
use jk_tests::sensor::{Logger, Rng, sample};
use jk_tests::{check, check_close};

/// Metres of head in one bar (998.2 kg/m^3, g = 9.81).
const BAR: f64 = 1e5 / (998.2 * 9.81);

const NETWORK: &str = "[TITLE]
The test estate, as its consultant's EPANET file

[JUNCTIONS]
;ID   Elev  Demand
S1    0     0
T     0     0
S2    0     0
S3    0     0

[RESERVOIRS]
;ID   Head
R     60

[PIPES]
;ID   Node1  Node2  Length  Diameter  Roughness  MinorLoss  Status
P1    R      S1     400     150       0.1        0          Open
P2    S1     T      300     150       0.1        0          Open
P3    T      S2     500     150       0.1        0          Open
P4    T      S3     600     150       0.1        0          Open

[OPTIONS]
Units    LPS
Headloss D-W

[END]
";

const SITE: &str = "# The estate, as its engineer would describe it.
[site]
name     = Test estate
network  = estate.inp
zone     = +02:00
altitude = 1450
rating   = PN16

[logger S1 gatehouse]
node  = S1
file  = s1.csv
clock = gnss

[logger S2 north block]
node  = S2
file  = s2.csv
clock = gnss
unit  = bar        # its header names no unit

[logger S3 south block]
node  = S3
file  = s3.csv
clock = gnss
range = 0 16 bar

[ratings]
P4 = PN10

[power]
monitored = yes
";

/// A logger export: local time (no zone), 12:15:00 + `t`, bar.
fn export(name: &str, heads: &[f64], phase: f64, unit_in_header: bool) -> String {
    let unit = if unit_in_header { " (bar)" } else { "" };
    let mut s = format!("Logger,{name}\nRate,128 Hz\nTimestamp,Pressure{unit}\n");
    for (k, h) in heads.iter().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        let t = phase + k as f64 / 128.0;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let ms = (t * 1000.0).round() as u64;
        let _ = writeln!(
            s,
            "2026-09-29 12:15:{:02}.{:03},{:.4}",
            ms / 1000,
            ms % 1000,
            h / BAR
        );
    }
    s
}

/// Writes the site's files into `dir`.
fn write_site(dir: &Path) {
    let (sim, burst) = estate(Some(200.0));
    let mut net = sim.net;
    net.add_burst(Burst {
        node: burst.expect("split"),
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(3.0),
        opening: BurstOpening::Instantaneous,
    })
    .unwrap();
    let run = net
        .simulate(
            &InitialCondition::AtRest {
                head: Head::new(60.0),
            },
            Time::new(10.0),
            &sim.sensors.map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .unwrap();
    let dt = run.trace.time_step().value();
    std::fs::create_dir_all(dir).unwrap();
    let mut rng = Rng::new(61);
    for (i, name) in ["S1 gatehouse", "S2 north block", "S3 south block"]
        .iter()
        .enumerate()
    {
        let truth: Vec<f64> = run.trace.head(i).iter().map(|h| h.value()).collect();
        let logger = Logger {
            rate: 128.0,
            phase: rng.uniform() / 128.0,
            noise: 0.02,
        };
        let heads = sample(&truth, dt, &logger, &mut rng);
        std::fs::write(
            dir.join(format!("s{}.csv", i + 1)),
            export(name, &heads, logger.phase, i != 1),
        )
        .unwrap();
    }
    std::fs::write(dir.join("estate.inp"), NETWORK).unwrap();
    std::fs::write(dir.join("site.txt"), SITE).unwrap();
}

#[test]
fn vs11_a_site_file_and_its_recordings_replay_through_the_live_engine() {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("site_file");
    write_site(&dir);
    println!(
        "Wrote a site file, a network and three logger exports to {}",
        dir.display()
    );
    let loaded = jk_files::site::load(&dir.join("site.txt")).unwrap();
    println!(
        "Read '{}': {} loggers. The readers' notes:",
        loaded.name,
        loaded.site.sensors.len()
    );
    for n in &loaded.notes {
        println!("  - {n}");
    }
    check(
        "  local stamps without a zone are read as the site's time",
        loaded
            .notes
            .iter()
            .any(|n| n.contains("read as the site's local time")),
    );
    // ISA at 1450 m: 101 325 (1 - 0.0065 x 1450 / 288.15)^5.25588 = 85 076 Pa;
    // (2 339 - 85 076) / (998.2 x 9.80665) = -8.452 m.
    check_close(
        "  the vapour floor from 1450 m [m]",
        loaded.config.features.vapour_floor.value(),
        -8.452,
        0.001,
    );
    let rating = |k: usize| {
        loaded.site.network.pipes()[k]
            .pipe
            .rating()
            .map(Head::value)
    };
    check(
        "  ratings: P1-P3 PN16, P4 PN10",
        rating(0).is_some_and(|r| (r - 163.4).abs() < 0.1)
            && rating(3).is_some_and(|r| (r - 102.2).abs() < 0.1),
    );
    check(
        "  S3's range 0-16 bar",
        loaded.site.sensors[2]
            .range
            .is_some_and(|(lo, hi)| lo.value().abs() < 1e-9 && (hi.value() - 163.4).abs() < 0.1),
    );

    let mut sink = CollectingSink::default();
    let r = replay(
        &loaded.site,
        &loaded.config,
        &loaded.records,
        Time::new(1.0),
        &mut sink,
    )
    .unwrap();
    let latest = r.latest();
    for a in &latest {
        println!("{}", a.render(&loaded.site.pipe_names));
    }
    check("one event", latest.len() == 1);
    let a = latest[0];
    check("  an ALARM for a burst", a.level == Level::Alarm);
    let loc = a.location.as_ref().expect("located");
    check(
        "  on P4 (named from the network file)",
        loaded.site.pipe_names[loc.most_likely.pipe.index()] == "P4",
    );
    check_close(
        "  at 200 m along it",
        loc.most_likely.distance.value(),
        200.0,
        0.01,
    );
    check(
        "  at 12:15:03 local time, written so",
        a.render(&loaded.site.pipe_names)
            .contains("at 2026-09-29 12:15:03.0 UTC+02:00"),
    );
    check(
        "  each logger's fatigue loading counted",
        r.loading.len() == 3 && r.loading.iter().all(|l| l.total() > 0.0),
    );
}

#[test]
fn vs11_a_site_file_with_a_mistake_is_refused_with_its_line() {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("site_file_bad");
    write_site(&dir);
    println!("Site files with mistakes (each refused, saying where):");
    for (what, from, to) in [
        ("a typo in a key", "altitude = 1450", "altitud = 1450"),
        (
            "a logger on a node the network lacks",
            "node  = S3",
            "node  = S9",
        ),
        (
            "a rating without a unit",
            "rating   = PN16",
            "rating   = 160",
        ),
        ("an unknown section", "[power]", "[pumps]"),
    ] {
        std::fs::write(dir.join("site.txt"), SITE.replace(from, to)).unwrap();
        let e = jk_files::site::load(&dir.join("site.txt"));
        let said = e
            .as_ref()
            .map_or_else(ToString::to_string, |_| "accepted".into());
        println!("  {what}: {said}");
        check(
            &format!("  {what}: refused, with its line"),
            e.as_ref().is_err_and(|e| e.line.is_some()),
        );
    }
}
