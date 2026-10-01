//! Site files: one plain-text file that says what a site is - its network
//! file, its loggers (where each sits, its export, its range and clock),
//! its pipes' ratings, its time zone and altitude, and the pump power
//! losses on record. [`load`] reads it and everything it names, and builds
//! what the live engine needs.
//!
//! ```text
//! # A joukowsky site file. Paths are relative to this file.
//! [site]
//! name     = Riverside estate
//! network  = riverside.inp        # EPANET network file
//! zone     = +02:00               # local time minus UTC (South Africa: +02:00)
//! altitude = 1450                 # m above sea level: where water boils
//! rating   = PN10                 # every pipe, unless listed under [ratings]
//! dates    = day-first            # 01/02/2026 is 1 February (or month-first)
//!
//! [logger S1 gatehouse]           # the name used in alerts
//! node   = J-12                   # the network node it sits on
//! file   = s1.csv                 # its export
//! column = Pressure               # which channel, if the file has several
//! unit   = bar                    # for a column whose header names none
//! range  = 0 16 bar               # its measuring range
//! clock  = gnss                   # gnss (1 us), ntp (40 ms) or seconds
//!
//! [ratings]
//! P-17 = PN16                     # or e.g. 160 m, 16 bar
//!
//! [power]
//! monitored = yes                 # pump power is logged at this site
//! lost      = 2026-10-01 14:02:00 # each recorded loss (local time unless zoned)
//! ```
//!
//! Strict and loud, like the other readers: an unknown section or key is
//! an error (a typo must not pass silently); what is assumed - a clock not
//! stated, times without a zone - is a note.

use std::path::Path;

use jk_application::{Feed, PipelineConfig, SensorSite, Site};
use jk_classification::Context;
use jk_hydraulics::pipe::pn;
use jk_hydraulics::vapour::{VAPOUR_PRESSURE_20C, vapour_floor};
use jk_kernel::{Head, Length, STANDARD_GRAVITY, Time};

use crate::epanet::{self, InpOptions};
use crate::error::ReadError;
use crate::logger::{self, CsvOptions, PressureUnit};
use crate::note::Note;

/// A site file, read with everything it names.
#[derive(Debug, Clone)]
pub struct LoadedSite {
    /// The site's name.
    pub name: String,
    /// The site as the engine knows it.
    pub site: Site,
    /// The pipeline settings the site implies (its vapour floor).
    pub config: PipelineConfig,
    /// Each logger's record, in sensor order, ready for
    /// [`jk_application::replay`].
    pub records: Vec<Feed>,
    /// Everything assumed or approximated, from every file, prefixed with
    /// the file it concerns.
    pub notes: Vec<String>,
}

/// One `key = value` line.
struct Entry {
    line: usize,
    key: String,
    value: String,
}

/// One `[section]` and its entries.
struct Section {
    line: usize,
    kind: String,
    name: String,
    entries: Vec<Entry>,
}

/// Reads the site file at `path` and the network and logger files it
/// names.
///
/// # Errors
/// A file that cannot be read, a line that breaks the format, an unknown
/// section or key, a logger on a node the network does not have.
///
/// # Panics
/// Never: pipe and node indices come from the network just read.
#[allow(clippy::too_many_lines)] // one site, read top to bottom
pub fn load(path: &Path) -> Result<LoadedSite, ReadError> {
    let text = std::fs::read_to_string(path)
        .map_err(|e| ReadError::general(format!("cannot read {}: {e}", path.display())))?;
    let base = path.parent().unwrap_or_else(|| Path::new("."));
    let sections = parse(&text)?;
    let mut notes = Vec::new();

    // [site]
    let site_section = sections
        .iter()
        .find(|s| s.kind == "site")
        .ok_or_else(|| ReadError::general("no [site] section"))?;
    let (mut name, mut network_file, mut zone, mut altitude, mut default_rating) =
        (String::from("site"), None, None, None, None);
    let mut day_first = true;
    for e in &site_section.entries {
        match e.key.as_str() {
            "name" => name.clone_from(&e.value),
            "network" => network_file = Some(base.join(&e.value)),
            "zone" => {
                zone = Some(parse_zone(&e.value).ok_or_else(|| bad(e, "a zone like +02:00"))?);
            }
            "altitude" => {
                altitude = Some(
                    e.value
                        .parse::<f64>()
                        .map_err(|_| bad(e, "metres above sea level"))?,
                );
            }
            "rating" => {
                default_rating = Some(
                    parse_rating(&e.value)
                        .ok_or_else(|| bad(e, "a rating like PN10, 16 bar or 160 m"))?,
                );
            }
            "dates" => {
                day_first = match e.value.as_str() {
                    "day-first" => true,
                    "month-first" => false,
                    _ => return Err(bad(e, "day-first or month-first")),
                };
            }
            _ => return Err(unknown(e, "[site]")),
        }
    }
    let network_file = network_file
        .ok_or_else(|| ReadError::at(site_section.line, "[site] needs network = FILE.inp"))?;

    // The network.
    let inp = epanet::read(&network_file, &InpOptions::default())
        .map_err(|e| ReadError::general(format!("{}: {e}", network_file.display())))?;
    let mut network = inp.network.clone();
    notes.extend(prefixed(&network_file, &inp.notes));
    if let Some(r) = default_rating {
        for k in 0..network.pipes().len() {
            let id = network.pipe_id(k).expect("in range");
            network
                .set_pipe_rating(id, r)
                .map_err(|e| ReadError::general(e.to_string()))?;
        }
    }

    // [ratings]
    for s in sections.iter().filter(|s| s.kind == "ratings") {
        for e in &s.entries {
            let k = inp
                .pipe_ids
                .iter()
                .position(|id| *id == e.key)
                .ok_or_else(|| {
                    ReadError::at(e.line, format!("no pipe '{}' in the network", e.key))
                })?;
            let r = parse_rating(&e.value)
                .ok_or_else(|| bad(e, "a rating like PN10, 16 bar or 160 m"))?;
            network
                .set_pipe_rating(network.pipe_id(k).expect("in range"), r)
                .map_err(|e| ReadError::general(e.to_string()))?;
        }
    }

    // [power]
    let mut context = Context::unknown();
    for s in sections.iter().filter(|s| s.kind == "power") {
        for e in &s.entries {
            match e.key.as_str() {
                "monitored" => {
                    context.power_monitored = match e.value.as_str() {
                        "yes" => true,
                        "no" => false,
                        _ => return Err(bad(e, "yes or no")),
                    };
                }
                "lost" => {
                    let stamp = logger::time::parse(&e.value, day_first)
                        .ok_or_else(|| bad(e, "a date and time"))?;
                    let utc = if stamp.zoned {
                        stamp.seconds
                    } else {
                        stamp.seconds - zone.unwrap_or(0.0)
                    };
                    context.power_losses.push(Time::new(utc));
                    context.power_monitored = true;
                }
                _ => return Err(unknown(e, "[power]")),
            }
        }
    }

    // [logger NAME]
    let mut sensors = Vec::new();
    let mut records = Vec::new();
    for s in sections.iter().filter(|s| s.kind == "logger") {
        if s.name.is_empty() {
            return Err(ReadError::at(
                s.line,
                "a logger needs a name: [logger S1 gatehouse]",
            ));
        }
        let (mut node, mut file, mut column, mut unit, mut range, mut clock) =
            (None, None, None, None, None, None);
        for e in &s.entries {
            match e.key.as_str() {
                "node" => node = Some((e.line, e.value.clone())),
                "file" => file = Some(base.join(&e.value)),
                "column" => column = Some(e.value.clone()),
                "unit" => {
                    unit = Some(
                        PressureUnit::parse(&e.value)
                            .ok_or_else(|| bad(e, "bar, kPa, psi, m, ..."))?,
                    );
                }
                "range" => {
                    range =
                        Some(parse_range(&e.value).ok_or_else(|| bad(e, "a range like 0 16 bar"))?);
                }
                "clock" => {
                    clock = Some(match e.value.as_str() {
                        "gnss" => 1e-6,
                        "ntp" => 0.04,
                        v => v
                            .parse::<f64>()
                            .ok()
                            .filter(|c| *c > 0.0)
                            .ok_or_else(|| bad(e, "gnss, ntp or seconds"))?,
                    });
                }
                _ => return Err(unknown(e, "[logger]")),
            }
        }
        let (node_line, node_id) = node
            .ok_or_else(|| ReadError::at(s.line, format!("logger {} needs node = ID", s.name)))?;
        let k = inp
            .node_ids
            .iter()
            .position(|id| *id == node_id)
            .ok_or_else(|| {
                ReadError::at(node_line, format!("no node '{node_id}' in the network"))
            })?;
        let file = file.ok_or_else(|| {
            ReadError::at(s.line, format!("logger {} needs file = EXPORT.csv", s.name))
        })?;
        let clock = clock.unwrap_or_else(|| {
            notes.push(format!(
                "logger {}: no clock stated - assumed synchronised over the internet (NTP, 40 ms)",
                s.name
            ));
            0.04
        });
        let recording = logger::read(
            &file,
            &CsvOptions {
                unit,
                day_first,
                ..CsvOptions::default()
            },
        )
        .map_err(|e| ReadError::general(format!("{}: {e}", file.display())))?;
        notes.extend(prefixed(&file, &recording.notes));
        let channel = match &column {
            Some(c) => recording
                .channels
                .iter()
                .position(|ch| {
                    ch.name
                        .to_ascii_lowercase()
                        .contains(&c.to_ascii_lowercase())
                })
                .ok_or_else(|| {
                    ReadError::general(format!("{}: no column like '{c}'", file.display()))
                })?,
            None if recording.channels.len() == 1 => 0,
            None => {
                return Err(ReadError::at(
                    s.line,
                    format!(
                        "{} has {} pressure columns: say which with column = NAME",
                        file.display(),
                        recording.channels.len()
                    ),
                ));
            }
        };
        let shift = if recording.zoned {
            0.0
        } else if let Some(z) = zone {
            notes.push(format!(
                "logger {}: timestamps without a zone - read as the site's local time",
                s.name
            ));
            -z
        } else {
            notes.push(format!(
                "logger {}: timestamps without a zone and no site zone - taken as UTC",
                s.name
            ));
            0.0
        };
        let sensor = sensors.len();
        sensors.push(SensorSite {
            node: network.node_id(k).expect("in range"),
            name: s.name.clone(),
            clock: None,
            uncalibrated_clock: Time::new(clock),
            range,
        });
        records.push(Feed {
            sensor,
            interval: recording.interval,
            runs: recording
                .runs
                .iter()
                .map(|r| (Time::new(r.start.value() + shift), r.heads[channel].clone()))
                .collect(),
        });
    }
    if sensors.is_empty() {
        return Err(ReadError::general(
            "no [logger ...] sections: a site needs loggers",
        ));
    }
    for s in &sections {
        if !matches!(s.kind.as_str(), "site" | "logger" | "ratings" | "power") {
            return Err(ReadError::at(
                s.line,
                format!("unknown section [{}]", s.kind),
            ));
        }
    }

    let mut config = PipelineConfig::default();
    match altitude {
        Some(z) => {
            config.features.vapour_floor =
                vapour_floor(Length::new(z), VAPOUR_PRESSURE_20C, STANDARD_GRAVITY);
        }
        None => notes.push("no altitude: water taken to boil at -10.1 m (sea level)".into()),
    }
    if zone.is_none() {
        notes.push("no zone: alert times are UTC".into());
    }
    let site = Site {
        network,
        sensors,
        pipe_names: inp.pipe_ids.clone(),
        context,
        schedule: None,
        utc_offset: zone.map(Time::new),
    };
    Ok(LoadedSite {
        name,
        site,
        config,
        records,
        notes,
    })
}

/// The sections of a site file.
fn parse(text: &str) -> Result<Vec<Section>, ReadError> {
    let mut sections: Vec<Section> = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let line = i + 1;
        let content = raw.split('#').next().unwrap_or("").trim();
        if content.is_empty() {
            continue;
        }
        if let Some(inner) = content.strip_prefix('[') {
            let inner = inner
                .strip_suffix(']')
                .ok_or_else(|| ReadError::at(line, "a section header ends with ]"))?
                .trim();
            let (kind, name) = inner.split_once(' ').unwrap_or((inner, ""));
            sections.push(Section {
                line,
                kind: kind.to_ascii_lowercase(),
                name: name.trim().to_string(),
                entries: Vec::new(),
            });
            continue;
        }
        let (key, value) = content
            .split_once('=')
            .ok_or_else(|| ReadError::at(line, "expected key = value"))?;
        let section = sections
            .last_mut()
            .ok_or_else(|| ReadError::at(line, "a value before any [section]"))?;
        section.entries.push(Entry {
            line,
            key: key.trim().to_string(),
            value: value.trim().to_string(),
        });
    }
    Ok(sections)
}

fn bad(e: &Entry, wanted: &str) -> ReadError {
    ReadError::at(
        e.line,
        format!("{} = '{}': expected {wanted}", e.key, e.value),
    )
}

fn unknown(e: &Entry, section: &str) -> ReadError {
    ReadError::at(e.line, format!("unknown key '{}' in {section}", e.key))
}

fn prefixed(file: &Path, notes: &[Note]) -> Vec<String> {
    let name = file.file_name().map_or_else(
        || file.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    notes
        .iter()
        .map(|n| match n.line {
            Some(l) => format!("{name} line {l}: {}", n.text),
            None => format!("{name}: {}", n.text),
        })
        .collect()
}

/// "+02:00", "-05:30", "+2", "UTC", "UTC+2" -> seconds.
fn parse_zone(s: &str) -> Option<f64> {
    let s = s.trim();
    let s = s.strip_prefix("UTC").unwrap_or(s).trim();
    if s.is_empty() {
        return Some(0.0);
    }
    let (sign, rest) = match s.as_bytes()[0] {
        b'+' => (1.0, &s[1..]),
        b'-' => (-1.0, &s[1..]),
        _ => return None,
    };
    let (h, m) = rest.split_once(':').unwrap_or((rest, "0"));
    let (h, m) = (h.parse::<f64>().ok()?, m.parse::<f64>().ok()?);
    (h <= 14.0 && m < 60.0).then_some(sign * (h * 3600.0 + m * 60.0))
}

/// "PN10", "PN 16", "16 bar", "160 m" -> a gauge head.
fn parse_rating(s: &str) -> Option<Head> {
    let s = s.trim();
    if let Some(rest) = s.strip_prefix("PN").or_else(|| s.strip_prefix("pn")) {
        return rest.trim().parse::<f64>().ok().filter(|b| *b > 0.0).map(pn);
    }
    let (number, unit) = s.split_once(' ')?;
    let value: f64 = number.parse().ok()?;
    let unit = PressureUnit::parse(unit.trim())?;
    let head = value * unit.metres_head(998.2, STANDARD_GRAVITY.value());
    (head > 0.0).then(|| Head::new(head))
}

/// "0 16 bar" -> (lowest, highest) gauge heads.
fn parse_range(s: &str) -> Option<(Head, Head)> {
    let parts: Vec<&str> = s.split_whitespace().collect();
    let [lo, hi, unit] = parts.as_slice() else {
        return None;
    };
    let unit = PressureUnit::parse(unit)?;
    let m = unit.metres_head(998.2, STANDARD_GRAVITY.value());
    let (lo, hi) = (lo.parse::<f64>().ok()? * m, hi.parse::<f64>().ok()? * m);
    (hi > lo).then(|| (Head::new(lo), Head::new(hi)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zones_ratings_and_ranges() {
        assert_eq!(parse_zone("+02:00"), Some(7200.0));
        assert_eq!(parse_zone("UTC+2"), Some(7200.0));
        assert_eq!(parse_zone("-05:30"), Some(-19_800.0));
        assert_eq!(parse_zone("UTC"), Some(0.0));
        assert_eq!(parse_zone("SAST"), None);
        assert!((parse_rating("PN10").unwrap().value() - 102.16).abs() < 0.01);
        assert!((parse_rating("16 bar").unwrap().value() - 163.4).abs() < 0.1);
        assert!((parse_rating("160 m").unwrap().value() - 160.0).abs() < 1e-9);
        assert!(
            parse_rating("160").is_none(),
            "a number without a unit is refused"
        );
        let (lo, hi) = parse_range("0 16 bar").unwrap();
        assert!(lo.value().abs() < 1e-12 && (hi.value() - 163.4).abs() < 0.1);
    }
}
