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
use jk_localisation::Localisation;

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
    /// Names of the sensors that saw it, in order of arrival.
    pub sensors: Vec<String>,
    /// Plain-language reasons: classification evidence and pipeline notes.
    pub reasons: Vec<String>,
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
            "[{:?}] #{}{revision} {} ({:.0}% confident) at t = {:.3} s",
            self.level,
            self.id,
            self.class.to_string().to_uppercase(),
            self.confidence * 100.0,
            self.time.value()
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
        let _ = write!(out, "\n  seen by: {}", self.sensors.join(", "));
        for r in &self.reasons {
            let _ = write!(out, "\n  because: {r}");
        }
        out
    }
}
