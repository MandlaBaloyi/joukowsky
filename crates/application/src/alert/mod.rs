//! What the engine tells people.
//!
//! One alert per event, carrying everything needed to act and to audit:
//! what happened (class, confidence), where (most likely point, the 90%
//! region, the top pipes), when, which sensors saw it, and WHY (principle
//! P5). Its level follows docs rule E1: low-confidence events are logged
//! for review, not pushed as alarms.

use std::fmt::Write as _;

use jk_classification::EventClass;
use jk_kernel::Time;
use jk_kernel::clock::stamp;
use jk_localisation::{Localisation, SizeEstimate};

/// How loudly an alert is raised.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Normal operation, logged only (e.g. a valve closure).
    Info,
    /// A person should look: the engine is unsure, or only one sensor saw
    /// it (possibly a sensor fault).
    Review,
    /// A damaging surge (e.g. a pump trip): notify operations.
    Warning,
    /// A confident, located burst: act now.
    Alarm,
}

/// One event, as reported.
#[derive(Debug, Clone, PartialEq)]
pub struct Alert {
    /// Running number, the same across revisions of one event.
    pub id: u32,
    /// 0 when first issued; raised each time late data changes the alert
    /// (e.g. a sensor whose upload was delayed saw the event too).
    pub revision: u32,
    /// How loudly to raise it.
    pub level: Level,
    /// What it was (`Unknown` if the classifier abstained).
    pub class: EventClass,
    /// Confidence in that class.
    pub confidence: f64,
    /// When the event started: the emission time from localisation, or the
    /// first arrival if it could not be located.
    pub time: Time,
    /// Where, if at least two sensors saw it.
    pub location: Option<Localisation>,
    /// How big: the flow a located burst lets out, from the drops the
    /// loggers measured (`jk_localisation::size`).
    pub size: Option<SizeEstimate>,
    /// Names of the sensors that saw it, in order of arrival.
    pub sensors: Vec<String>,
    /// Plain-language reasons: classification evidence and pipeline notes.
    pub reasons: Vec<String>,
    /// The site's local time minus UTC, if known (from its schedule): times
    /// are shown as its local clock, else as UTC.
    pub utc_offset: Option<Time>,
}

impl Alert {
    /// A human-readable summary, with pipes named by `pipe_names`.
    #[must_use]
    pub fn render(&self, pipe_names: &[String]) -> String {
        let name = |i: usize| {
            pipe_names
                .get(i)
                .cloned()
                .unwrap_or_else(|| format!("pipe {i}"))
        };
        let revision = if self.revision > 0 {
            format!(" (revision {})", self.revision)
        } else {
            String::new()
        };
        let mut out = format!(
            "[{:?}] #{}{revision} {} ({:.0}% confident) at {}",
            self.level,
            self.id,
            self.class.to_string().to_uppercase(),
            self.confidence * 100.0,
            stamp(self.time.value(), self.utc_offset.map(Time::value))
        );
        match &self.location {
            Some(loc) => {
                let _ = write!(
                    out,
                    "\n  where: {} at {:.1} m from its start; 90% within {:.0} m of pipe",
                    name(loc.most_likely.pipe.index()),
                    loc.most_likely.distance.value(),
                    loc.credible_length.value()
                );
                for p in loc.pipes.iter().take(3) {
                    let _ = write!(
                        out,
                        "\n    {:<20} {:>5.1}%",
                        name(p.pipe.index()),
                        p.probability * 100.0
                    );
                }
            }
            None => out.push_str("\n  where: not located (fewer than two sensors saw it)"),
        }
        if let Some(s) = &self.size {
            let _ = write!(
                out,
                "\n  size: about {:.2} l/s (+/- {:.2} l/s), from {} logger(s)",
                s.flow.value() * 1000.0,
                s.uncertainty.value() * 1000.0,
                s.loggers
            );
        }
        let _ = write!(out, "\n  seen by: {}", self.sensors.join(", "));
        for r in &self.reasons {
            let _ = write!(out, "\n  because: {r}");
        }
        out
    }
}
