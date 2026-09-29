//! The pages of the viewer, one per menu entry.
//!
//! - [`overview`]: headline figures, the event timeline, the alerts.
//! - [`alerts`]: every alert in full - map, arrival at each logger, why.
//! - [`loggers`]: each logger's record, zoomable, one crosshair for all.
//! - [`network`]: the site's pipes and nodes.
//! - [`placement`]: where loggers should go (three pages).
//! - [`about`]: what the data is and what the viewer does not show yet.

pub mod about;
pub mod alerts;
pub mod loggers;
pub mod network;
pub mod overview;
pub mod placement;

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

/// The data the browser draws its charts from: every logger's record
/// (full resolution), fronts and impaired stretches.
pub fn data_json(event: &EventModel) -> String {
    let mut s = String::from(r#"{"records":["#);
    for (i, r) in event.records.iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        let fronts: Vec<f64> = r.fronts.iter().map(|f| f.arrival).collect();
        let _ = write!(
            s,
            r#"{{"name":{},"start":{:.6},"dt":{:.9},"h":{},"fronts":{},"impaired":["#,
            json_str(&r.name),
            r.start,
            r.interval,
            json_numbers(&r.samples, 3),
            json_numbers(&fronts, 4)
        );
        for (k, (a, b)) in r.impaired.iter().enumerate() {
            let _ = write!(s, "{}[{a:.4},{b:.4}]", if k > 0 { "," } else { "" });
        }
        s.push_str("]}");
    }
    s.push_str("]}");
    s
}
