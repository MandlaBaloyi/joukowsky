//! Supply cuts as alerts: sensors whose pipes stood empty, grouped into one
//! incident, raised when confirmed and revised when the water comes back
//! (`docs/04_detection_and_localisation/event_detection.txt` section 6).
//!
//! One logger reading about 0 could be a disconnected transducer, so a cut
//! seen by one sensor is raised for Review; two or more agreeing make it a
//! Warning. Times are each sensor's own clock: supply changes last
//! minutes, far longer than any clock error.

use jk_classification::EventClass;
use jk_kernel::Time;

use crate::alert::{Alert, Level};

/// Cuts confirmed at sensors within this long of each other are one
/// incident [s].
pub(super) const GROUP: f64 = 900.0;

/// One sensor's part in a supply incident [s, its clock].
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Member {
    pub sensor: usize,
    /// When its pipe emptied (`None`: before its data began).
    pub from: Option<f64>,
    /// When it had been empty long enough to call it.
    pub confirmed: f64,
    /// When the supply came back.
    pub until: Option<f64>,
    /// When its data stopped while still empty.
    pub data_stopped: Option<f64>,
}

impl Member {
    /// Still empty, as far as its data goes.
    pub fn open(&self) -> bool {
        self.until.is_none() && self.data_stopped.is_none()
    }
}

/// A supply cut across one or more sensors.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct Incident {
    pub id: u32,
    /// Revision to send next (0 until first sent).
    pub revision: u32,
    pub members: Vec<Member>,
    /// Changed since last sent ...
    pub stale: bool,
    /// ... last at this time [s]: sent once every sensor's data is past it
    /// (one alert for sensors changing together, not one each).
    pub changed_at: f64,
}

impl Incident {
    /// Whether a cut confirmed at `confirmed` belongs here.
    pub fn takes(&self, confirmed: f64) -> bool {
        self.members.iter().any(Member::open)
            && self
                .members
                .iter()
                .all(|m| (m.confirmed - confirmed).abs() <= GROUP)
    }

    /// The alert as it stands, sensors named by `names`.
    pub fn alert(&self, names: &[String]) -> Alert {
        let minutes = |a: f64, b: f64| (b - a) / 60.0;
        let mut reasons = Vec::new();
        for m in &self.members {
            let name = &names[m.sensor];
            let since = m.from.map_or_else(
                || "since before its data began".to_string(),
                |f| format!("since t = {f:.0} s"),
            );
            reasons.push(format!(
                "{name}: the pressure has stood at about 0 {since} - its pipe is EMPTY: supply lost"
            ));
            if let Some(back) = m.until {
                let off = m.from.map_or_else(
                    || format!("at least {:.0} min", minutes(m.confirmed, back)),
                    |f| format!("{:.0} min", minutes(f, back)),
                );
                reasons.push(format!(
                    "{name}: supply RETURNED at t = {back:.0} s, after {off} - the pipes refill; \
                     expect refill surges"
                ));
            }
            if let Some(stop) = m.data_stopped {
                reasons.push(format!(
                    "{name}: its data stopped at t = {stop:.0} s while the pipe was still empty - \
                     whether the supply has come back since is not known"
                ));
            }
        }
        if self.members.len() == 1 {
            reasons.push(
                "seen by ONE sensor only: a supply cut - or a sensor fault (a disconnected \
                 transducer also reads about 0); a second logger on the same supply would tell"
                    .into(),
            );
        }
        if self.members.iter().all(|m| m.until.is_some()) {
            reasons.push("supply back everywhere it was lost".into());
        }
        let start = self
            .members
            .iter()
            .map(|m| m.from.unwrap_or(m.confirmed))
            .fold(f64::INFINITY, f64::min);
        Alert {
            id: self.id,
            revision: self.revision,
            level: if self.members.len() >= 2 {
                Level::Warning
            } else {
                Level::Review
            },
            class: EventClass::SupplyCut,
            confidence: 1.0,
            time: Time::new(start),
            location: None,
            size: None,
            sensors: self
                .members
                .iter()
                .map(|m| names[m.sensor].clone())
                .collect(),
            reasons,
        }
    }
}
