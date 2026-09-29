//! Delimited text to a table: separator, decimal comma, preamble, header.

use super::time;
use crate::error::ReadError;

/// One data row.
#[derive(Debug, Clone, PartialEq)]
pub struct Row {
    /// 1-based line number.
    pub line: usize,
    /// Its fields, trimmed.
    pub fields: Vec<String>,
}

/// The file as a table.
#[derive(Debug, Clone, PartialEq)]
pub struct Table {
    /// Column names (generated as "column N" when the file has none).
    pub header: Vec<String>,
    /// Whether the file had a header row.
    pub has_header: bool,
    /// Lines of metadata above the header (logger serial, settings...).
    pub preamble: usize,
    /// The column separator.
    pub delimiter: char,
    /// Numbers written with a decimal comma (`4,52`).
    pub decimal_comma: bool,
    /// How many leading columns make the timestamp (2: date and time).
    pub time_columns: usize,
    /// The data rows, in file order.
    pub rows: Vec<Row>,
}

/// Tokens that mean "no value".
const MISSING: [&str; 10] = [
    "", "nan", "na", "n/a", "#n/a", "null", "none", "-", "--", "?",
];

/// A field's value: a number, missing, or not a number at all.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value {
    /// A number.
    Number(f64),
    /// Marked as no value (empty, NaN, N/A, ...).
    Missing,
    /// Neither a number nor a known 'no value' mark.
    Bad,
}

/// Reads a data field.
#[must_use]
pub fn value(field: &str, decimal_comma: bool) -> Value {
    let f = field.trim().trim_matches('"').trim();
    if MISSING.contains(&f.to_ascii_lowercase().as_str()) {
        return Value::Missing;
    }
    let parsed = if decimal_comma {
        f.replace(',', ".").parse::<f64>()
    } else {
        f.parse::<f64>()
    };
    match parsed {
        Ok(v) if v.is_finite() => Value::Number(v),
        Ok(_) => Value::Missing,
        Err(_) => Value::Bad,
    }
}

/// Splits a line on `delimiter`, honouring double quotes.
fn fields(line: &str, delimiter: char) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut quoted = false;
    for c in line.chars() {
        match c {
            '"' => quoted = !quoted,
            c if c == delimiter && !quoted => out.push(std::mem::take(&mut cur).trim().to_string()),
            c => cur.push(c),
        }
    }
    out.push(cur.trim().to_string());
    out
}

/// The separator that gives most lines the same number (at least two) of
/// fields; on a tie `;` beats tab beats `,` (a `;` file with decimal
/// commas also splits evenly on `,`).
fn delimiter(lines: &[(usize, &str)]) -> Option<char> {
    let sample: Vec<&str> = lines.iter().rev().take(30).map(|(_, l)| *l).collect();
    let mut best: Option<(usize, char)> = None;
    for d in [';', '\t', ','] {
        let counts: Vec<usize> = sample.iter().map(|l| fields(l, d).len()).collect();
        let mut modal = (0, 0);
        for &c in &counts {
            let n = counts.iter().filter(|&&x| x == c).count();
            if n > modal.1 {
                modal = (c, n);
            }
        }
        if modal.0 >= 2 && best.is_none_or(|(n, _)| modal.1 > n) {
            best = Some((modal.1, d));
        }
    }
    best.map(|(_, d)| d)
}

/// How many leading fields make a timestamp in this row: 1, 2 (date and
/// time in separate columns), or none.
fn timestamp_width(f: &[String], day_first: bool) -> Option<usize> {
    let first = f.first()?;
    if !first.contains(':')
        && f.get(1).is_some_and(|s| s.contains(':'))
        && time::parse(&format!("{first} {}", f[1]), day_first).is_some()
    {
        return Some(2);
    }
    time::parse(first, day_first).map(|_| 1)
}

/// Splits `text` into a table.
///
/// # Errors
/// No separator that gives at least two columns, or no row that starts
/// with a timestamp followed by a value.
pub fn split(text: &str, day_first: bool) -> Result<Table, ReadError> {
    let lines: Vec<(usize, &str)> = text
        .lines()
        .enumerate()
        .map(|(i, l)| (i + 1, l.trim_end_matches('\r')))
        .filter(|(_, l)| !l.trim().is_empty())
        .collect();
    let delimiter = delimiter(&lines).ok_or_else(|| {
        ReadError::general(
            "no column separator found (tried ';', tab and ','): is this a logger export?",
        )
    })?;
    let split: Vec<(usize, Vec<String>)> = lines
        .iter()
        .map(|(n, l)| (*n, fields(l, delimiter)))
        .collect();
    // The first data row: a timestamp, then at least one value.
    let decimal_comma = delimiter != ',';
    let is_data = |f: &[String]| {
        timestamp_width(f, day_first).is_some_and(|w| {
            f.len() > w && f[w..].iter().any(|x| value(x, decimal_comma) != Value::Bad)
        })
    };
    let first = split
        .iter()
        .position(|(_, f)| is_data(f))
        .ok_or_else(|| ReadError::general("no row starts with a timestamp followed by a value"))?;
    let time_columns = timestamp_width(&split[first].1, day_first).unwrap_or(1);
    let width = split[first].1.len();
    let has_header = first > 0 && split[first - 1].1.len() == width;
    let header = if has_header {
        split[first - 1].1.clone()
    } else {
        (1..=width).map(|i| format!("column {i}")).collect()
    };
    let preamble = if has_header { first - 1 } else { first };
    // Decimal comma: only if a data field actually looks like "4,52".
    let decimal_comma = decimal_comma
        && split[first..].iter().take(50).any(|(_, f)| {
            f[time_columns.min(f.len())..].iter().any(|x| {
                let t = x.trim();
                t.contains(',') && t.replace(',', ".").parse::<f64>().is_ok()
            })
        });
    let rows = split[first..]
        .iter()
        .map(|(n, f)| Row {
            line: *n,
            fields: f.clone(),
        })
        .collect();
    Ok(Table {
        header,
        has_header,
        preamble,
        delimiter,
        decimal_comma,
        time_columns,
        rows,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plain_csv_with_a_preamble() {
        let t = split(
            "Logger SN 1234\nRate 128 Hz\nTime,P1 (bar)\n2026-09-29 10:00:00.000,4.51\n2026-09-29 10:00:00.008,4.52\n",
            true,
        )
        .unwrap();
        assert_eq!(t.delimiter, ',');
        assert_eq!(t.preamble, 2);
        assert!(t.has_header && t.header == ["Time", "P1 (bar)"]);
        assert_eq!(t.rows.len(), 2);
        assert_eq!(t.rows[0].line, 4);
        assert!(!t.decimal_comma);
    }

    #[test]
    fn semicolons_decimal_commas_and_split_date_time() {
        let t = split(
            "Date;Time;Druck [kPa]\n29.09.2026;10:00:00,000;451,2\n29.09.2026;10:00:00,125;451,3\n",
            true,
        )
        .unwrap();
        assert_eq!(t.delimiter, ';');
        assert!(t.decimal_comma);
        assert_eq!(t.time_columns, 2);
        assert_eq!(value("451,3", true), Value::Number(451.3));
    }

    #[test]
    fn values_missing_and_bad() {
        assert_eq!(value("", false), Value::Missing);
        assert_eq!(value("NaN", false), Value::Missing);
        assert_eq!(value("#N/A", false), Value::Missing);
        assert_eq!(value("err", false), Value::Bad);
        assert_eq!(value(" 4.5 ", false), Value::Number(4.5));
    }
}
