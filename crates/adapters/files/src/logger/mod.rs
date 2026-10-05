//! Pressure recordings exported by loggers (CSV and similar text).
//!
//! A logger export is a table: a timestamp, then one or more pressure
//! columns, often under a few lines of metadata. This reader turns it into
//! what the engine consumes - for each pressure channel, continuous runs
//! of evenly spaced heads in metres - and a list of NOTES for everything it
//! had to decide or repair (principle P6: gaps stay gaps, bad values
//! become gaps, nothing is filled in).
//!
//! Strict where a wrong guess would be silent:
//! - the unit must be named in the column header (`P1 (bar)`, `head_m`) or
//!   given by the caller - it is never guessed (a bar/kPa mix-up is a
//!   factor of 100);
//! - `01/02/2026` is read day first (South African and European exports)
//!   unless the caller says otherwise;
//! - columns that are not pressures (temperature, battery, flow) are left
//!   out, and named.
//!
//! - [`table`]: separator, decimal comma, preamble, header, date/time
//!   columns.
//! - [`time`]: timestamps to seconds (own calendar arithmetic).
//! - [`units`]: pressure units to metres of head.
//! - [`grid`]: rows to a regular grid, split where rows are missing.

pub mod grid;
pub mod table;
pub mod time;
pub mod units;

use std::path::Path;

use jk_kernel::Time;

use crate::error::ReadError;
use crate::note::Note;
use table::Value;
pub use units::PressureUnit;

/// Choices the file may not make for itself.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CsvOptions {
    /// Unit for pressure columns whose header names none.
    pub unit: Option<PressureUnit>,
    /// Read `01/02/2026` as 1 February (true) or 2 January.
    pub day_first: bool,
    /// Water density for pressure-to-head [kg/m^3] (998.2: 20 C).
    pub density: f64,
    /// Gravity [m/s^2].
    pub gravity: f64,
}

impl Default for CsvOptions {
    fn default() -> Self {
        Self {
            unit: None,
            day_first: true,
            density: 998.2,
            gravity: 9.81,
        }
    }
}

/// One pressure channel of a recording.
#[derive(Debug, Clone, PartialEq)]
pub struct Channel {
    /// The column header.
    pub name: String,
    /// The unit it was recorded in.
    pub unit: PressureUnit,
}

/// A continuous run: evenly spaced samples, no rows missing between them.
#[derive(Debug, Clone, PartialEq)]
pub struct Run {
    /// Time of the first sample [s since 1970-01-01, or the file's own
    /// origin for plain-number times].
    pub start: Time,
    /// Heads [m] for each channel, one per sample; NaN where the file had
    /// no valid value.
    pub heads: Vec<Vec<f64>>,
}

/// A logger recording, read.
#[derive(Debug, Clone, PartialEq)]
pub struct Recording {
    /// The pressure channels, in column order.
    pub channels: Vec<Channel>,
    /// Seconds between samples.
    pub interval: Time,
    /// The first timestamp as the file wrote it.
    pub first_stamp: String,
    /// The continuous runs, in time order.
    pub runs: Vec<Run>,
    /// Data rows read.
    pub rows: usize,
    /// Whether the timestamps named a time zone (then times are UTC);
    /// false: times are as written - the site's local clock.
    pub zoned: bool,
    /// Everything decided or repaired.
    pub notes: Vec<Note>,
}

impl Recording {
    /// Total samples (per channel).
    #[must_use]
    pub fn samples(&self) -> usize {
        self.runs
            .iter()
            .map(|r| r.heads.first().map_or(0, Vec::len))
            .sum()
    }
}

fn note(notes: &mut Vec<Note>, line: Option<usize>, text: impl Into<String>) {
    notes.push(Note {
        line,
        text: text.into(),
    });
}

/// Words that mark a column as something other than a pressure.
fn not_a_pressure(header: &str) -> Option<&'static str> {
    let h = header.to_ascii_lowercase();
    [
        ("temp", "temperature"),
        ("°c", "temperature"),
        ("(c)", "temperature"),
        ("[c]", "temperature"),
        ("batt", "battery"),
        ("volt", "voltage"),
        ("(v)", "voltage"),
        ("[v]", "voltage"),
        ("flow", "flow"),
        ("l/s", "flow"),
        ("m3/h", "flow"),
        ("signal", "signal strength"),
        ("rssi", "signal strength"),
        ("%", "a percentage"),
    ]
    .iter()
    .find(|(k, _)| h.contains(k))
    .map(|(_, w)| *w)
}

/// Reads a logger export.
///
/// # Errors
/// No timestamped rows, fewer than two increasing timestamps, an
/// unreadable timestamp, no pressure column, or a pressure column whose
/// unit is neither named in its header nor given in `options`.
#[allow(clippy::too_many_lines)] // one reading, told top to bottom
pub fn parse(text: &str, options: &CsvOptions) -> Result<Recording, ReadError> {
    let t = table::split(text, options.day_first)?;
    let mut notes = Vec::new();
    if t.preamble > 0 {
        note(
            &mut notes,
            None,
            format!("{} line(s) of metadata above the table skipped", t.preamble),
        );
    }
    if t.decimal_comma {
        note(&mut notes, None, "numbers use a decimal comma");
    }

    // Channels: every column after the timestamp that is a pressure.
    let mut channels = Vec::new();
    let mut columns = Vec::new();
    let mut missing_unit = Vec::new();
    for (c, name) in t.header.iter().enumerate().skip(t.time_columns) {
        if let Some(what) = not_a_pressure(name) {
            note(
                &mut notes,
                None,
                format!("column '{name}' looks like {what}, not a pressure: not used"),
            );
            continue;
        }
        let unit = if t.has_header {
            PressureUnit::from_header(name)
        } else {
            None
        };
        match unit.or(options.unit) {
            Some(u) => {
                if unit.is_none() {
                    note(
                        &mut notes,
                        None,
                        format!("column '{name}' names no unit: {} as instructed", u.name()),
                    );
                }
                channels.push(Channel {
                    name: name.clone(),
                    unit: u,
                });
                columns.push(c);
            }
            None => missing_unit.push(name.clone()),
        }
    }
    if !missing_unit.is_empty() {
        return Err(ReadError::general(format!(
            "no unit for column(s) {}: the header must name it (e.g. 'P1 (bar)') or it must be given - units are never guessed",
            missing_unit.join(", ")
        )));
    }
    if channels.is_empty() {
        return Err(ReadError::general("no pressure column found"));
    }

    // Timestamps.
    let mut times = Vec::with_capacity(t.rows.len());
    let mut zoned = None;
    for r in &t.rows {
        let stamp_text = r.fields[..t.time_columns.min(r.fields.len())].join(" ");
        let stamp = time::parse(&stamp_text, options.day_first)
            .ok_or_else(|| ReadError::at(r.line, format!("'{stamp_text}' is not a timestamp")))?;
        zoned.get_or_insert(stamp.zoned);
        times.push(stamp.seconds);
    }
    let g = grid::regularise(&times)
        .ok_or_else(|| ReadError::general("fewer than two rows with increasing timestamps"))?;
    let rate = 1.0 / g.interval;
    note(
        &mut notes,
        None,
        if g.whole_rate && g.interval > 1.0 {
            format!("one sample every {:.0} s", g.interval)
        } else if g.whole_rate {
            format!("{rate:.0} sample(s) a second")
        } else {
            format!("{rate:.4} samples a second (not a whole rate: the logger's clock may drift)")
        },
    );
    match zoned {
        Some(true) => note(
            &mut notes,
            None,
            "timestamps carry a time zone: times are UTC",
        ),
        _ => note(
            &mut notes,
            None,
            "timestamps carry no time zone: times are as written (only differences matter within one file; across loggers their clocks must agree)",
        ),
    }
    if g.jitter > 0.05 * g.interval {
        note(
            &mut notes,
            None,
            format!(
                "timestamps sit up to {:.2} ms off the logger's regular clock (rounding): samples are placed on the regular clock",
                g.jitter * 1000.0
            ),
        );
    }
    if g.spans.len() > 1 {
        note(
            &mut notes,
            None,
            format!(
                "{} gap(s) in the rows - {} sample(s), {:.3} s in all, never arrived: the recording is split there, nothing is filled in",
                g.spans.len() - 1,
                g.missing,
                to_f64(g.missing) * g.interval
            ),
        );
    }
    if !g.dropped.is_empty() {
        note(
            &mut notes,
            Some(t.rows[g.dropped[0]].line),
            format!(
                "{} row(s) repeat an earlier time or go back in time: left out",
                g.dropped.len()
            ),
        );
    }

    // Values to heads, run by run.
    let factors: Vec<f64> = channels
        .iter()
        .map(|c| c.unit.metres_head(options.density, options.gravity))
        .collect();
    let mut bad = vec![0usize; channels.len()];
    let mut empty = vec![0usize; channels.len()];
    let runs = g
        .spans
        .iter()
        .map(|span| Run {
            start: Time::new(span.start),
            heads: columns
                .iter()
                .enumerate()
                .map(|(ch, &col)| {
                    span.rows
                        .iter()
                        .map(|&i| {
                            let f = t.rows[i].fields.get(col).map_or("", String::as_str);
                            match table::value(f, t.decimal_comma) {
                                Value::Number(v) => v * factors[ch],
                                Value::Missing => {
                                    empty[ch] += 1;
                                    f64::NAN
                                }
                                Value::Bad => {
                                    bad[ch] += 1;
                                    f64::NAN
                                }
                            }
                        })
                        .collect()
                })
                .collect(),
        })
        .collect();
    for (ch, c) in channels.iter().enumerate() {
        note(
            &mut notes,
            None,
            format!(
                "'{}': {} -> metres of head (x {:.4}); ground elevation not added",
                c.name,
                c.unit.name(),
                factors[ch]
            ),
        );
        if empty[ch] + bad[ch] > 0 {
            note(
                &mut notes,
                None,
                format!(
                    "'{}': {} empty and {} unreadable value(s) - kept as gaps, not filled",
                    c.name, empty[ch], bad[ch]
                ),
            );
        }
    }
    Ok(Recording {
        channels,
        interval: Time::new(g.interval),
        first_stamp: t.rows[0].fields[..t.time_columns.min(t.rows[0].fields.len())].join(" "),
        runs,
        rows: t.rows.len(),
        zoned: zoned == Some(true),
        notes,
    })
}

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}

/// Reads a logger export file.
///
/// # Errors
/// A file that cannot be read, or the errors of [`parse`].
pub fn read(path: &Path, options: &CsvOptions) -> Result<Recording, ReadError> {
    let bytes = std::fs::read(path)
        .map_err(|e| ReadError::general(format!("cannot read {}: {e}", path.display())))?;
    parse(&String::from_utf8_lossy(&bytes), options)
}
