//! The pages of the viewer, one per menu entry.
//!
//! - [`overview`]: headline figures, the event timeline, the alerts.
//! - [`alerts`]: every alert in full - map, arrival at each logger, why.
//! - [`loggers`]: each logger's record, zoomable, one crosshair for all.
//! - [`network`]: the site's pipes and nodes.
//! - [`placement`]: where loggers should go (three pages).
//! - [`file`]: a network read from an EPANET `.inp` file, with the reader's notes.
//! - [`recording`]: logger recordings read from files - real pressure data.
//! - [`about`]: what the data is and what the viewer does not show yet.

pub mod about;
pub mod alerts;
pub mod file;
pub mod loggers;
pub mod network;
pub mod overview;
pub mod placement;
pub mod recording;

use std::fmt::Write as _;

use jk_application::{Alert, Level};

use super::model::EventModel;
use super::svg::{json_numbers, json_str};

/// A level's label and CSS class.
pub fn status(level: Level) -> (&'static str, &'static str) {
    match level {
        Level::Alarm => ("Alarm", "alarm"),
        Level::Warning => ("Warning", "warning"),
        Level::Review => ("Review", "review"),
        Level::Info => ("Info", "info"),
    }
}

/// A status badge: coloured dot AND the word (never colour alone).
pub fn badge(level: Level) -> String {
    let (label, class) = status(level);
    format!(r#"<span class="badge"><span class="dot {class}"></span>{label}</span>"#)
}

/// "BURST" -> "Burst".
pub fn class_name(alert: &Alert) -> String {
    let s = alert.class.to_string();
    let mut c = s.chars();
    c.next().map_or(String::new(), |f| {
        f.to_uppercase().collect::<String>() + &c.as_str().to_lowercase()
    })
}

/// The eight-step sequential legend, with words at its ends.
pub fn ramp_legend(what: &str, low: &str, high: &str) -> String {
    format!(
        r#"<span class="key">{what}: {low} <span class="ramp"><span class="q1"></span><span class="q2"></span><span class="q3"></span><span class="q4"></span><span class="q5"></span><span class="q6"></span><span class="q7"></span><span class="q8"></span></span> {high}</span>"#
    )
}

/// One chart line for the browser: a named record in continuous runs,
/// with the fronts the detector picked and the stretches it was not
/// watching at full sensitivity. Times in seconds on the page's axis.
pub struct ChartRecord<'a> {
    pub name: String,
    pub interval: f64,
    /// (start, samples) of each continuous run.
    pub runs: Vec<(f64, &'a [f64])>,
    pub fronts: Vec<f64>,
    pub impaired: Vec<(f64, f64)>,
}

/// The data the browser draws its charts from: named sets of records
/// (full resolution), e.g. "event" for the demo, "rec" for recordings.
pub fn data_json(sets: &[(&str, Vec<ChartRecord<'_>>)]) -> String {
    let mut s = String::from(r#"{"sets":{"#);
    for (n, (name, records)) in sets.iter().enumerate() {
        if n > 0 {
            s.push(',');
        }
        let _ = write!(s, "{}:[", json_str(name));
        for (i, r) in records.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            let _ = write!(
                s,
                r#"{{"name":{},"dt":{:.9},"fronts":{},"runs":["#,
                json_str(&r.name),
                r.interval,
                json_numbers(&r.fronts, 4)
            );
            for (k, (start, h)) in r.runs.iter().enumerate() {
                let _ = write!(
                    s,
                    r#"{}{{"start":{start:.6},"h":{}}}"#,
                    if k > 0 { "," } else { "" },
                    json_numbers(h, 3)
                );
            }
            s.push_str(r#"],"impaired":["#);
            for (k, (a, b)) in r.impaired.iter().enumerate() {
                let _ = write!(s, "{}[{a:.4},{b:.4}]", if k > 0 { "," } else { "" });
            }
            s.push_str("]}");
        }
        s.push(']');
    }
    s.push_str("}}");
    s
}

/// The demo event's records, as chart records.
pub fn event_records(event: &EventModel) -> Vec<ChartRecord<'_>> {
    event
        .records
        .iter()
        .map(|r| ChartRecord {
            name: r.name.clone(),
            interval: r.interval,
            runs: vec![(r.start, r.samples.as_slice())],
            fronts: r.fronts.iter().map(|f| f.arrival).collect(),
            impaired: r.impaired.clone(),
        })
        .collect()
}
