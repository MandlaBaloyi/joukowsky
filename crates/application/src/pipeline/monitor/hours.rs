//! Pumps and valves hour by hour (gap G-19): a PRV active by day may be
//! shut at night, a pump running in the evening stopped at 03:00. Which
//! loggers can hear a burst, which routes its wave takes and how much of it
//! arrives all depend on that - so the monitor locates, corroborates and
//! sizes each event with the devices as they stand at its hour.
//!
//! With the site's [`Schedule`], the extended-period run is made once, for
//! two days from local midnight (the first lets tanks settle into their
//! daily cycle); the second day's state at each whole hour gives that
//! hour's devices ([`Devices::from_steady`]). Without a schedule - or for
//! hours the run does not reach (supply lost) - the devices are as the
//! base steady state leaves them, and the alert says so.

use jk_hydraulics::steady::controls::DAY;
use jk_kernel::Time;
use jk_localisation::Devices;
use jk_localisation::graph::TravelGraph;

use crate::error::ApplicationError;
use crate::pipeline::{Schedule, Site};

/// The device states of each hour of the local day.
#[derive(Debug, Clone)]
pub(super) struct Hours {
    /// Distinct device states; 0 is the base steady state.
    pub sets: Vec<Devices>,
    /// Fastest wave travel time between every pair of sensors under each
    /// set [s].
    pub travel: Vec<Vec<Vec<f64>>>,
    /// For each local hour, the set in force.
    pub by_hour: [usize; 24],
    /// Why some hours fall back to the base state, if any do.
    pub note: Option<String>,
}

impl Hours {
    /// The hours of `site`.
    ///
    /// # Errors
    /// A schedule the extended-period run refuses (bad options or
    /// controls).
    pub fn of(site: &Site) -> Result<Self, ApplicationError> {
        let mut sets = vec![Devices::of(&site.network)];
        let mut by_hour = [0; 24];
        let mut note = None;
        if let Some(schedule) = &site.schedule {
            let mut period = schedule.period.clone();
            period.duration = Time::new(2.0 * DAY);
            let mut seen: Vec<(f64, Devices)> = Vec::new();
            site.network
                .extended_observed(&period, &mut |t, s| {
                    if t.value() >= DAY - 1e-6 {
                        seen.push((t.value(), Devices::from_steady(&site.network, s)));
                    }
                })
                .map_err(|e| ApplicationError::InvalidInput(format!("the site's schedule: {e}")))?;
            let mut missing = Vec::new();
            for (hour, slot) in by_hour.iter_mut().enumerate() {
                #[allow(clippy::cast_precision_loss)] // under 24
                let mark = DAY + hour as f64 * 3600.0;
                match seen.iter().find(|(t, _)| *t >= mark - 1e-6) {
                    Some((_, d)) => {
                        *slot = sets.iter().position(|x| x == d).unwrap_or_else(|| {
                            sets.push(d.clone());
                            sets.len() - 1
                        });
                    }
                    None => missing.push(hour),
                }
            }
            if !missing.is_empty() {
                note = Some(format!(
                    "the site's schedule loses supply before {:02}:00: from then on pumps and \
                     valves are taken as at base demand",
                    missing[0]
                ));
            }
        }
        let travel = sets
            .iter()
            .map(|d| {
                let graph = TravelGraph::with_devices(&site.network, d);
                let pipe_times: Vec<f64> = site
                    .network
                    .pipes()
                    .iter()
                    .map(|p| p.pipe.travel_time().value())
                    .collect();
                site.sensors
                    .iter()
                    .map(|a| {
                        let times = graph.times_from(a.node.index(), &pipe_times);
                        site.sensors.iter().map(|b| times[b.node.index()]).collect()
                    })
                    .collect()
            })
            .collect();
        Ok(Self {
            sets,
            travel,
            by_hour,
            note,
        })
    }

    /// The set in force at UTC time `t` [s] on a site `utc_offset` [s] from
    /// UTC (unknown: hours taken as UTC, and the words say so), and words
    /// for the alert (None without a schedule).
    pub fn at(
        &self,
        schedule: Option<&Schedule>,
        utc_offset: Option<f64>,
        t: f64,
    ) -> (usize, Option<String>) {
        if schedule.is_none() {
            return (0, None);
        }
        let local = (t + utc_offset.unwrap_or(0.0)).rem_euclid(DAY);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 0..24
        let hour = ((local / 3600.0) as usize).min(23);
        let k = self.by_hour[hour];
        let shut = self.sets[k].shut_count();
        let mut words = format!(
            "located with pumps and valves as the site's schedule has them at {hour:02}:00{}{}",
            if utc_offset.is_none() {
                " (taken as UTC: the site's time zone is not set)"
            } else {
                ""
            },
            match shut {
                0 => String::new(),
                1 => " (1 shut: no wave passes it)".into(),
                n => format!(" ({n} shut: no wave passes them)"),
            }
        );
        if let Some(n) = &self.note {
            words.push_str("; ");
            words.push_str(n);
        }
        (k, Some(words))
    }
}
