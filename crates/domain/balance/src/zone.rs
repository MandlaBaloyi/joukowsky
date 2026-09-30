//! The balance of a zone: what its meters say is used inside it.
//!
//! ```text
//!   used(t) = sum of inflows - sum of outflows - dV/dt
//! ```
//!
//! dV/dt is the change of water stored in the zone's tanks (taken as the
//! central difference of the volume series, one-sided at the ends). A
//! reading any meter is missing (NaN) makes that balance missing too -
//! never guessed (P6).

use jk_kernel::Time;

use crate::BalanceError;

/// Which way a meter's flow counts for the zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Water entering the zone.
    In,
    /// Water leaving the zone.
    Out,
}

/// A flow meter on the zone's boundary and its readings.
#[derive(Debug, Clone, PartialEq)]
pub struct Meter {
    /// Name, for reports.
    pub name: String,
    /// Which way it counts.
    pub direction: Direction,
    /// Flow readings [m^3/s], one per interval of the zone's [`Series`];
    /// NaN where missing.
    pub flows: Vec<f64>,
}

/// A regular series: a value every `interval` from `start`.
#[derive(Debug, Clone, PartialEq)]
pub struct Series {
    /// Time of the first value.
    pub start: Time,
    /// Time between values.
    pub interval: Time,
    /// The values (NaN where missing).
    pub values: Vec<f64>,
}

/// The zone's balance [m^3/s] at each reading, from its `meters` (all on
/// the grid `start` + k `interval`) and, if the zone stores water, the
/// stored `volume` [m^3] on the same grid.
///
/// # Errors
/// No meters, a non-positive interval, or series of different lengths.
pub fn balance(
    start: Time,
    interval: Time,
    meters: &[Meter],
    volume: Option<&[f64]>,
) -> Result<Series, BalanceError> {
    let dt = interval.value();
    if !(dt.is_finite() && dt > 0.0) {
        return Err(BalanceError::InvalidInput(format!(
            "interval {dt} s: must be positive"
        )));
    }
    let Some(first) = meters.first() else {
        return Err(BalanceError::InvalidInput("a zone needs meters".into()));
    };
    let n = first.flows.len();
    if meters.iter().any(|m| m.flows.len() != n) || volume.is_some_and(|v| v.len() != n) {
        return Err(BalanceError::InvalidInput(
            "every meter and the volume need one value per interval".into(),
        ));
    }
    let values = (0..n)
        .map(|k| {
            let metered: f64 = meters
                .iter()
                .map(|m| match m.direction {
                    Direction::In => m.flows[k],
                    Direction::Out => -m.flows[k],
                })
                .sum();
            let stored = volume.map_or(0.0, |v| {
                if n < 2 {
                    return 0.0;
                }
                let (a, b) = (k.saturating_sub(1), (k + 1).min(n - 1));
                #[allow(clippy::cast_precision_loss)] // 1 or 2
                let span = (b - a) as f64 * dt;
                (v[b] - v[a]) / span
            });
            metered - stored
        })
        .collect();
    Ok(Series {
        start,
        interval,
        values,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inflows_minus_outflows_minus_storage() {
        let meters = vec![
            Meter {
                name: "in".into(),
                direction: Direction::In,
                flows: vec![0.030, 0.030, 0.030],
            },
            Meter {
                name: "out".into(),
                direction: Direction::Out,
                flows: vec![0.010, 0.010, f64::NAN],
            },
        ];
        // The tank gains 60 m^3 over the two 10-minute steps: 0.05 m^3/s.
        let volume = [100.0, 130.0, 160.0];
        let b = balance(Time::ZERO, Time::new(600.0), &meters, Some(&volume)).unwrap();
        assert!((b.values[0] - (0.020 - 0.05)).abs() < 1e-15);
        assert!((b.values[1] - (0.020 - 0.05)).abs() < 1e-15);
        assert!(b.values[2].is_nan(), "a missing reading stays missing");
    }
}
