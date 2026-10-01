//! Timestamps as loggers write them, to seconds.
//!
//! Handled: `2026-09-29 10:15:03.125`, `2026-09-29T10:15:03.125Z`,
//! `2026/09/29 10:15:03`, day-first `29/09/2026 10:15:03` (South African
//! and European exports) or month-first when asked, `29.09.2026`, a zone
//! (`Z`, `+02:00`, `-0500`) or none, `AM`/`PM`; and plain numbers (seconds,
//! or milliseconds when they are epoch-sized).
//!
//! Dates become days by Howard Hinnant's `days_from_civil` algorithm (the
//! proleptic Gregorian calendar, exact, no tables), so a timestamp becomes
//! seconds since 1970-01-01 - UTC when the file gives a zone, "as written"
//! when it does not (the reader notes which).

/// A parsed timestamp: seconds since 1970-01-01, and whether a zone was
/// given (so the value is true UTC).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stamp {
    /// Seconds since 1970-01-01 (or the file's own origin for numbers).
    pub seconds: f64,
    /// Whether a time zone was given.
    pub zoned: bool,
}

pub use jk_kernel::clock::days_from_civil;

fn days_in_month(year: i64, month: u32) -> u32 {
    match month {
        4 | 6 | 9 | 11 => 30,
        2 if (year % 4 == 0 && year % 100 != 0) || year % 400 == 0 => 29,
        2 => 28,
        _ => 31,
    }
}

/// Parses a date: year first (`2026-09-29`, `2026/09/29`) or year last
/// (`29/09/2026`, `29.09.2026`, `29-09-2026`), day first unless
/// `day_first` is false.
fn date(s: &str, day_first: bool) -> Option<(i64, u32, u32)> {
    let parts: Vec<&str> = s.split(['-', '/', '.']).collect();
    if parts.len() != 3
        || parts
            .iter()
            .any(|p| p.is_empty() || !p.bytes().all(|b| b.is_ascii_digit()))
    {
        return None;
    }
    let (y, m, d) = if parts[0].len() == 4 {
        (parts[0], parts[1], parts[2])
    } else if parts[2].len() == 4 {
        if day_first {
            (parts[2], parts[1], parts[0])
        } else {
            (parts[2], parts[0], parts[1])
        }
    } else {
        return None;
    };
    let (y, m, d): (i64, u32, u32) = (y.parse().ok()?, m.parse().ok()?, d.parse().ok()?);
    ((1..=12).contains(&m) && d >= 1 && d <= days_in_month(y, m)).then_some((y, m, d))
}

/// Parses `HH:MM[:SS[.fff]]`, returning seconds into the day.
fn clock(s: &str) -> Option<f64> {
    let parts: Vec<&str> = s.split(':').collect();
    if !(2..=3).contains(&parts.len()) {
        return None;
    }
    let h: u32 = parts[0].parse().ok()?;
    let m: u32 = parts[1].parse().ok()?;
    let sec: f64 = match parts.get(2) {
        Some(p) => {
            let p = p.replace(',', ".");
            if p.starts_with('.') || p.ends_with('.') {
                return None;
            }
            p.parse().ok()?
        }
        None => 0.0,
    };
    (h < 24 && m < 60 && (0.0..61.0).contains(&sec))
        .then(|| f64::from(h) * 3600.0 + f64::from(m) * 60.0 + sec)
}

/// Splits a trailing zone off a time: `Z`, `+HH:MM`, `+HHMM`, `-HH`.
/// Returns the rest and the zone's offset from UTC in seconds.
fn zone(s: &str) -> (&str, Option<f64>) {
    if let Some(rest) = s.strip_suffix('Z').or_else(|| s.strip_suffix('z')) {
        return (rest, Some(0.0));
    }
    if let Some(i) = s.rfind(['+', '-']) {
        let (rest, z) = s.split_at(i);
        if rest.contains(':') {
            let sign = if z.starts_with('-') { -1.0 } else { 1.0 };
            let digits: String = z[1..].chars().filter(char::is_ascii_digit).collect();
            let (h, m) = match digits.len() {
                2 => (digits.parse::<f64>().ok(), Some(0.0)),
                4 => (digits[..2].parse().ok(), digits[2..].parse().ok()),
                _ => (None, None),
            };
            if let (Some(h), Some(m)) = (h, m) {
                return (rest, Some(sign * (h * 3600.0 + m * 60.0)));
            }
        }
    }
    (s, None)
}

/// Parses one timestamp. `day_first` settles `01/02/2026`.
#[must_use]
pub fn parse(text: &str, day_first: bool) -> Option<Stamp> {
    let t = text.trim().trim_matches('"').trim();
    if t.is_empty() {
        return None;
    }
    // A plain number: seconds, or epoch milliseconds when that large.
    if let Ok(v) = t.parse::<f64>() {
        if !v.is_finite() {
            return None;
        }
        let seconds = if v.abs() > 1e11 { v / 1000.0 } else { v };
        return Some(Stamp {
            seconds,
            zoned: false,
        });
    }
    let upper = t.to_ascii_uppercase();
    let (body, pm) = if let Some(b) = upper.strip_suffix("PM") {
        (b.trim_end().to_string(), Some(true))
    } else if let Some(b) = upper.strip_suffix("AM") {
        (b.trim_end().to_string(), Some(false))
    } else {
        (upper, None)
    };
    let (d, rest) = match body.find(['T', ' ']) {
        Some(i) => (&body[..i], body[i + 1..].trim()),
        None => (body.as_str(), ""),
    };
    let (y, m, day) = date(d, day_first)?;
    let (clock_text, offset) = zone(rest);
    let mut secs = if clock_text.is_empty() {
        0.0
    } else {
        clock(clock_text.trim())?
    };
    if let Some(pm) = pm {
        let hour = (secs / 3600.0).floor();
        if !(1.0..=12.0).contains(&hour) {
            return None;
        }
        if pm && hour < 12.0 {
            secs += 12.0 * 3600.0;
        } else if !pm && hour >= 12.0 {
            secs -= 12.0 * 3600.0;
        }
    }
    #[allow(clippy::cast_precision_loss)] // days since 1970 are far below 2^52
    let days = days_from_civil(y, m, day) as f64;
    Some(Stamp {
        seconds: days * 86_400.0 + secs - offset.unwrap_or(0.0),
        zoned: offset.is_some(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn secs(t: &str) -> f64 {
        parse(t, true).unwrap().seconds
    }

    #[test]
    fn the_calendar_is_exact() {
        assert_eq!(days_from_civil(1970, 1, 1), 0);
        assert_eq!(days_from_civil(2000, 3, 1), 11_017);
        assert_eq!(days_from_civil(2026, 9, 29), 20_725);
        assert_eq!(days_from_civil(1969, 12, 31), -1);
    }

    #[test]
    fn the_usual_logger_formats() {
        // 2026-09-29 10:15:03.125 UTC = day 20725.
        let expected = 20_725.0 * 86_400.0 + 10.0 * 3600.0 + 15.0 * 60.0 + 3.125;
        for t in [
            "2026-09-29 10:15:03.125",
            "2026-09-29T10:15:03.125Z",
            "2026/09/29 10:15:03.125",
            "29/09/2026 10:15:03.125",
            "29.09.2026 10:15:03,125",
            "\"2026-09-29 10:15:03.125\"",
            "29/09/2026 10:15:03.125 AM",
        ] {
            assert!((secs(t) - expected).abs() < 1e-6, "{t}: {}", secs(t));
        }
        // A zone moves the instant to UTC.
        assert!((secs("2026-09-29T12:15:03.125+02:00") - expected).abs() < 1e-6);
        assert!(parse("2026-09-29T12:15:03+02:00", true).unwrap().zoned);
        assert!(!parse("2026-09-29 12:15:03", true).unwrap().zoned);
        // PM, and month-first when asked.
        assert!((secs("29/09/2026 10:15:03.125 PM") - expected - 12.0 * 3600.0).abs() < 1e-6);
        let us = parse("09/29/2026 10:15:03.125", false).unwrap().seconds;
        assert!((us - expected).abs() < 1e-6);
    }

    #[test]
    fn numbers_and_nonsense() {
        assert!((secs("12.5") - 12.5).abs() < 1e-12);
        assert!((secs("1790000000000") - 1_790_000_000.0).abs() < 1e-3);
        for bad in [
            "",
            "yesterday",
            "2026-13-01 10:00",
            "2026-02-30",
            "31/04/2026",
            "10:00",
        ] {
            assert!(parse(bad, true).is_none(), "{bad}");
        }
    }
}
