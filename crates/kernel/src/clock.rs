//! Calendar time: seconds since 1970-01-01 (UTC) to dates and back, and
//! the way the engine writes a moment for people.
//!
//! Dates by Howard Hinnant's `days_from_civil` / `civil_from_days`
//! algorithms - the proleptic Gregorian calendar, exact, no tables. Real
//! records carry seconds since 1970 (the logger reader makes them);
//! simulations start at 0. A time this large or larger is read as a date.

/// Seconds since 1970 from which a time is shown as a date (1973-03-03).
pub const DATED_FROM: f64 = 1e8;

/// Days from 1970-01-01 to the given date (proleptic Gregorian).
#[must_use]
pub fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    let y = if month <= 2 { year - 1 } else { year };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let m = i64::from(month);
    let mp = if m > 2 { m - 3 } else { m + 9 };
    let doy = (153 * mp + 2) / 5 + i64::from(day) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The date (year, month, day) `days` after 1970-01-01.
#[must_use]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // in range by construction
pub fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let month = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let year = yoe + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

/// A moment for people: `t` [s] as a local date and time when it is a
/// date (`utc_offset` [s] from the site, or UTC when unknown), else as
/// "t = 12.3 s" (a simulation's clock).
#[must_use]
pub fn stamp(t: f64, utc_offset: Option<f64>) -> String {
    if !(t.is_finite() && t >= DATED_FROM) {
        return format!("t = {t:.1} s");
    }
    let offset = utc_offset.unwrap_or(0.0);
    let local = t + offset;
    let days = (local / 86_400.0).floor();
    let of_day = local - days * 86_400.0;
    #[allow(clippy::cast_possible_truncation)] // dates are far inside i64
    let (y, m, d) = civil_from_days(days as i64);
    // Tenths, carried so 59.96 s becomes the next minute, not "60.0".
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 0 <= tenths < 864 000
    let tenths = (of_day * 10.0).round() as u64;
    let (h, min, s) = (tenths / 36_000, tenths / 600 % 60, tenths % 600);
    let zone = match utc_offset {
        None => "UTC".to_string(),
        Some(o) => {
            #[allow(clippy::cast_possible_truncation)] // offsets are whole minutes
            let minutes = (o / 60.0).round() as i64;
            let sign = if minutes < 0 { '-' } else { '+' };
            format!(
                "UTC{sign}{:02}:{:02}",
                minutes.abs() / 60,
                minutes.abs() % 60
            )
        }
    };
    format!(
        "{y}-{m:02}-{d:02} {h:02}:{min:02}:{:02}.{} {zone}",
        s / 10,
        s % 10
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn days_and_dates_are_inverse() {
        assert_eq!(days_from_civil(1970, 1, 1), 0);
        assert_eq!(days_from_civil(2026, 9, 29), 20_725);
        for days in [-1, 0, 11_017, 20_725, 59, 60, 365 * 400] {
            let (y, m, d) = civil_from_days(days);
            assert_eq!(days_from_civil(y, m, d), days, "{y}-{m}-{d}");
        }
        assert_eq!(civil_from_days(-1), (1969, 12, 31));
        assert_eq!(civil_from_days(11_016), (2000, 2, 29));
    }

    #[test]
    fn a_moment_for_people() {
        // 1 790 000 000 s = day 20 717 (2026-09-21) + 51 200 s (14:13:20) UTC.
        assert_eq!(stamp(1_790_000_000.0, None), "2026-09-21 14:13:20.0 UTC");
        assert_eq!(
            stamp(1_790_000_000.0, Some(7200.0)),
            "2026-09-21 16:13:20.0 UTC+02:00"
        );
        assert_eq!(
            stamp(156.04, Some(7200.0)),
            "t = 156.0 s",
            "a simulation's clock"
        );
        // 59.96 s rounds to the next minute.
        assert_eq!(stamp(1_790_000_039.96, None), "2026-09-21 14:14:00.0 UTC");
    }
}
