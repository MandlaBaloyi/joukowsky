//! SCADA and meter exports: flows and levels every few minutes, as
//! delimited text (Tier 0 data - `docs/03_sensing_and_signal/
//! data_source_requirements.txt`).
//!
//! The same table reading as the logger reader (separators, decimal
//! commas, preambles, timestamps, a regular clock - `logger/`), but the
//! columns are flows or levels, not pressures, so nothing is turned into
//! head. The unit is never guessed: a column heading names it ("p227
//! (m3/h)") or the caller gives it. Rows that never arrived are kept as
//! gaps (NaN), never filled (P6). First used for L-Town's readings
//! (data/README.md, D2).

use std::path::Path;

use jk_kernel::Time;

use crate::error::ReadError;
use crate::logger::{grid, table, time};
use crate::note::Note;

/// A unit a SCADA column can be in, and its factor to SI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScadaUnit {
    /// Cubic metres an hour (flow).
    CubicMetresPerHour,
    /// Litres a second (flow).
    LitresPerSecond,
    /// Cubic metres a second (flow).
    CubicMetresPerSecond,
    /// Metres (a tank's level, or a head).
    Metres,
}

impl ScadaUnit {
    /// Reads a unit's name ("m3/h", "l/s", "m3/s", "m"), ignoring case and
    /// spaces.
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        let s: String = s
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect::<String>()
            .to_lowercase()
            .replace('³', "3");
        match s.as_str() {
            "m3/h" | "cmh" | "m3/hr" => Some(Self::CubicMetresPerHour),
            "l/s" | "lps" => Some(Self::LitresPerSecond),
            "m3/s" | "cms" => Some(Self::CubicMetresPerSecond),
            "m" => Some(Self::Metres),
            _ => None,
        }
    }

    /// The unit named in parentheses at the end of a heading, if any.
    fn from_heading(heading: &str) -> Option<Self> {
        let open = heading.rfind('(')?;
        let close = heading[open..].find(')')? + open;
        Self::parse(&heading[open + 1..close])
    }

    /// Its name.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::CubicMetresPerHour => "m3/h",
            Self::LitresPerSecond => "l/s",
            Self::CubicMetresPerSecond => "m3/s",
            Self::Metres => "m",
        }
    }

    /// Factor to SI (m^3/s for flows, m for levels).
    #[must_use]
    pub fn to_si(self) -> f64 {
        match self {
            Self::CubicMetresPerHour => 1.0 / 3600.0,
            Self::LitresPerSecond => 1e-3,
            Self::CubicMetresPerSecond | Self::Metres => 1.0,
        }
    }
}

/// How to read a SCADA file.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ScadaOptions {
    /// Unit for columns whose heading names none.
    pub unit: Option<ScadaUnit>,
    /// Read `01/02/2026` as 1 February (true) or 2 January.
    pub day_first: bool,
}

/// One column of a SCADA file.
#[derive(Debug, Clone, PartialEq)]
pub struct ScadaColumn {
    /// The column heading.
    pub name: String,
    /// Its unit in the file.
    pub unit: ScadaUnit,
    /// Values in SI, one per slot of the regular clock from `start`; NaN
    /// where no reading arrived or it was empty or unreadable.
    pub values: Vec<f64>,
}

/// A SCADA file, read.
#[derive(Debug, Clone, PartialEq)]
pub struct ScadaTable {
    /// Time of the first slot [s since 1970-01-01, or the file's own
    /// origin for plain-number times].
    pub start: Time,
    /// Seconds between readings.
    pub interval: Time,
    /// The columns.
    pub columns: Vec<ScadaColumn>,
    /// Everything decided, repaired or left out.
    pub notes: Vec<Note>,
}

impl ScadaTable {
    /// The column with this heading (without its unit), if any.
    #[must_use]
    pub fn column(&self, name: &str) -> Option<&ScadaColumn> {
        self.columns
            .iter()
            .find(|c| c.name == name || c.name.split('(').next().is_some_and(|n| n.trim() == name))
    }
}

/// Reads SCADA text.
///
/// # Errors
/// No rows with increasing timestamps, a timestamp that cannot be read, or
/// a column with no unit named or given.
pub fn parse(text: &str, options: &ScadaOptions) -> Result<ScadaTable, ReadError> {
    let t = table::split(text, options.day_first)?;
    let mut notes = Vec::new();
    let mut note = |text: String| {
        notes.push(Note { line: None, text });
    };
    if t.decimal_comma {
        note("numbers use a decimal comma".into());
    }
    let mut times = Vec::with_capacity(t.rows.len());
    for r in &t.rows {
        let stamp_text = r.fields[..t.time_columns.min(r.fields.len())].join(" ");
        let stamp = time::parse(&stamp_text, options.day_first)
            .ok_or_else(|| ReadError::at(r.line, format!("'{stamp_text}' is not a timestamp")))?;
        times.push(stamp.seconds);
    }
    let g = grid::regularise(&times)
        .ok_or_else(|| ReadError::general("fewer than two rows with increasing timestamps"))?;
    note(format!("one reading every {:.0} s", g.interval));
    if g.missing > 0 {
        note(format!(
            "{} reading(s) never arrived: kept as gaps, not filled",
            g.missing
        ));
    }
    if !g.dropped.is_empty() {
        note(format!(
            "{} row(s) repeat an earlier time or go back in time: left out",
            g.dropped.len()
        ));
    }
    let first = g.spans.first().map_or(0.0, |s| s.start);
    let slot = |at: f64| -> usize {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // at >= first
        let k = ((at - first) / g.interval).round() as usize;
        k
    };
    let slots = g.spans.last().map_or(0, |s| slot(s.start) + s.rows.len());
    let mut columns = Vec::new();
    let mut unnamed = Vec::new();
    for (c, name) in t.header.iter().enumerate().skip(t.time_columns) {
        let Some(unit) = ScadaUnit::from_heading(name).or(options.unit) else {
            unnamed.push(name.clone());
            continue;
        };
        let mut values = vec![f64::NAN; slots];
        let mut bad = 0;
        for span in &g.spans {
            let s0 = slot(span.start);
            for (k, &i) in span.rows.iter().enumerate() {
                let f = t.rows[i].fields.get(c).map_or("", String::as_str);
                match table::value(f, t.decimal_comma) {
                    table::Value::Number(v) => values[s0 + k] = v * unit.to_si(),
                    _ => bad += 1,
                }
            }
        }
        if bad > 0 {
            note(format!(
                "'{name}': {bad} empty or unreadable value(s) - kept as gaps"
            ));
        }
        columns.push(ScadaColumn {
            name: name.clone(),
            unit,
            values,
        });
    }
    if !unnamed.is_empty() {
        return Err(ReadError::general(format!(
            "no unit for column(s) {}: the heading must name it (e.g. 'p227 (m3/h)') or it must be given - units are never guessed",
            unnamed.join(", ")
        )));
    }
    if let Some(u) = options.unit {
        note(format!(
            "columns that name no unit are in {} as instructed",
            u.name()
        ));
    }
    Ok(ScadaTable {
        start: Time::new(first),
        interval: Time::new(g.interval),
        columns,
        notes,
    })
}

/// Reads a SCADA file.
///
/// # Errors
/// A file that cannot be read, or the errors of [`parse`].
pub fn read(path: &Path, options: &ScadaOptions) -> Result<ScadaTable, ReadError> {
    let bytes = std::fs::read(path)
        .map_err(|e| ReadError::general(format!("cannot read {}: {e}", path.display())))?;
    parse(&String::from_utf8_lossy(&bytes), options)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn l_town_style_flows_with_a_gap() {
        // Semicolons, decimal commas, every 5 minutes, one row missing.
        let text = "Timestamp;p227;p235\n\
                    2018-01-01 00:00:00;77,77;83,93\n\
                    2018-01-01 00:05:00;72,51;76,34\n\
                    2018-01-01 00:15:00;70,00;75,00\n\
                    2018-01-01 00:20:00;71,00;75,00\n\
                    2018-01-01 00:25:00;72,00;75,00\n";
        let opts = ScadaOptions {
            unit: Some(ScadaUnit::CubicMetresPerHour),
            ..ScadaOptions::default()
        };
        let t = parse(text, &opts).unwrap();
        assert!((t.interval.value() - 300.0).abs() < 1e-9);
        let p227 = &t.column("p227").unwrap().values;
        assert_eq!(p227.len(), 6);
        assert!((p227[0] - 77.77 / 3600.0).abs() < 1e-15);
        assert!(p227[2].is_nan(), "the missing 00:10 reading stays a gap");
        assert!((p227[3] - 70.0 / 3600.0).abs() < 1e-15);
    }

    #[test]
    fn a_unit_is_never_guessed() {
        let text = "t;level\n2018-01-01 00:00;3,5\n2018-01-01 00:05;3,6\n";
        assert!(parse(text, &ScadaOptions::default()).is_err());
        let named = "t;T1 (m)\n2018-01-01 00:00;3,5\n2018-01-01 00:05;3,6\n";
        let t = parse(named, &ScadaOptions::default()).unwrap();
        assert!((t.column("T1").unwrap().values[1] - 3.6).abs() < 1e-12);
    }
}
