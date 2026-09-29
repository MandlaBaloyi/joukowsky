//! Pictures drawn by the engine side, as inline SVG: the event timeline,
//! network maps, column charts. (Pressure traces are drawn in the browser,
//! so they can be zoomed - `assets/app.js`.)
//!
//! Colours are never written here: every mark carries a CSS class whose
//! colour the page defines once per theme (light and dark), so the charts
//! follow the reader's setting. Text is escaped; hover details ride on
//! `data-tip` attributes and are shown by the page's script with
//! `textContent` only.

use std::fmt::Write as _;

use jk_hydraulics::Network;

use super::svg::{Scale, esc, tick_label, ticks, to_f64};
use crate::sim::sites::Drawing;

/// An alert on the timeline.
pub struct TimelineAlert {
    pub id: u32,
    /// CSS status class suffix: alarm, warning, review, info.
    pub status: &'static str,
    /// Hover text, lines separated by '|'.
    pub tip: String,
    /// When the event started [s].
    pub start: f64,
    /// When the alert went out (data time) [s].
    pub sent: f64,
}

/// The event timeline: one lane for the alerts (a dot where each event
/// started, a line to when its alert went out; each links to its page),
/// then a lane per logger with a tick at every front it picked.
pub fn timeline(
    alerts: &[TimelineAlert],
    loggers: &[(String, Vec<f64>)],
    span: (f64, f64),
    width: f64,
) -> String {
    let (left, right, lane) = (150.0, 16.0, 30.0);
    let height = lane * to_f64(loggers.len() + 1) + 34.0;
    let sx = Scale::new(span, (left, width - right));
    let mut s = format!(
        r#"<svg class="chart timeline" viewBox="0 0 {width} {height}" role="img" aria-label="Event timeline">"#
    );
    let (x_ticks, step) = ticks(span.0, span.1, 8);
    for v in &x_ticks {
        let x = sx.at(*v);
        let _ = write!(
            s,
            r#"<line class="grid" x1="{x:.1}" x2="{x:.1}" y1="4" y2="{:.1}"/><text class="tick" x="{x:.1}" y="{:.1}" text-anchor="middle">{} s</text>"#,
            height - 26.0,
            height - 8.0,
            tick_label(*v, step)
        );
    }
    // Lanes.
    let lanes = std::iter::once("Alerts").chain(loggers.iter().map(|(n, _)| n.as_str()));
    for (i, name) in lanes.enumerate() {
        let y = 6.0 + lane * to_f64(i);
        let _ = write!(
            s,
            r#"<rect class="lane" x="{left}" y="{y:.1}" width="{:.1}" height="{:.1}" rx="4"/><text class="label{}" x="0" y="{:.1}">{}</text>"#,
            width - left - right,
            lane - 6.0,
            if i == 0 { " strong" } else { "" },
            y + lane / 2.0 + 1.0,
            esc(name)
        );
    }
    for a in alerts {
        let (x0, x1, y) = (sx.at(a.start), sx.at(a.sent), 6.0 + (lane - 6.0) / 2.0);
        let _ = write!(
            s,
            r##"<a href="#alerts/{id}"><line class="span" x1="{x0:.1}" x2="{x1:.1}" y1="{y:.1}" y2="{y:.1}" data-tip="{tip}"/><circle class="tl-dot tl-{st}" cx="{x0:.1}" cy="{y:.1}" r="7" data-tip="{tip}"/><text class="label strong" x="{:.1}" y="{:.1}">#{id}</text></a>"##,
            x1 + 6.0,
            y + 4.0,
            id = a.id,
            st = a.status,
            tip = esc(&a.tip)
        );
    }
    for (i, (name, fronts)) in loggers.iter().enumerate() {
        let y = 6.0 + lane * to_f64(i + 1);
        for f in fronts {
            let x = sx.at(*f);
            let _ = write!(
                s,
                r#"<line class="tl-front" x1="{x:.1}" x2="{x:.1}" y1="{:.1}" y2="{:.1}" data-tip="{}"/>"#,
                y + 4.0,
                y + lane - 10.0,
                esc(&format!("{name}|front at {f:.3} s"))
            );
        }
    }
    s.push_str("</svg>");
    s
}

/// Something drawn on a network map.
pub enum Marker {
    /// A logger: filled, ringed, labelled.
    Logger(usize),
    /// A place a logger could go but does not: small, hollow.
    Candidate(usize),
    /// A point on a pipe (pipe index, metres from its start), marked and
    /// labelled.
    Point {
        pipe: usize,
        distance: f64,
        label: String,
        truth: bool,
    },
    /// A stretch of pipe (pipe index, from, to in metres), drawn as a halo.
    Stretch { pipe: usize, from: f64, to: f64 },
}

/// Where things go on a map: plan metres to pixels.
struct Plan<'a> {
    network: &'a Network,
    drawing: &'a Drawing,
    pad: f64,
    x0: f64,
    y0: f64,
    y1: f64,
    k: f64,
    width: f64,
    height: f64,
}

impl<'a> Plan<'a> {
    fn new(network: &'a Network, drawing: &'a Drawing, width: f64) -> Self {
        let pad = 70.0;
        let (mut x0, mut x1, mut y0, mut y1) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for &(x, y) in drawing
            .positions
            .iter()
            .chain(drawing.bends.iter().flatten())
        {
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
        }
        let k = (width - 2.0 * pad) / (x1 - x0).max(1.0);
        let height = ((y1 - y0) * k + 2.0 * pad).max(160.0);
        Self {
            network,
            drawing,
            pad,
            x0,
            y0,
            y1,
            k,
            width,
            height,
        }
    }

    fn at(&self, (x, y): (f64, f64)) -> (f64, f64) {
        // Map files have y pointing north; the page has y pointing down.
        let down = if self.drawing.y_up {
            self.y1 - y
        } else {
            y - self.y0
        };
        (self.pad + (x - self.x0) * self.k, self.pad + down * self.k)
    }

    fn ends(&self, p: usize) -> ((f64, f64), (f64, f64)) {
        let link = &self.network.pipes()[p];
        (
            self.at(self.drawing.positions[link.from.index()]),
            self.at(self.drawing.positions[link.to.index()]),
        )
    }

    /// The point `d` metres along pipe `p`.
    fn point(&self, p: usize, d: f64) -> (f64, f64) {
        let ((ax, ay), (bx, by)) = self.ends(p);
        let f = (d / self.network.pipes()[p].pipe.length().value()).clamp(0.0, 1.0);
        (ax + f * (bx - ax), ay + f * (by - ay))
    }
}

/// A network map: every pipe in its colour class, with a hover tip; nodes
/// named; markers on top.
pub fn network_map(
    network: &Network,
    drawing: &Drawing,
    pipe_class: &[&str],
    pipe_tip: &[String],
    markers: &[Marker],
    width: f64,
) -> String {
    let plan = Plan::new(network, drawing, width);
    let mut s = format!(
        r#"<svg class="chart map{}" viewBox="0 0 {width} {:.0}" role="img">"#,
        if drawing.positions.len() > 40 {
            " map-dense"
        } else {
            ""
        },
        plan.height
    );
    // Halos under the pipes.
    for m in markers {
        if let Marker::Stretch { pipe, from, to } = m {
            let (a, b) = (plan.point(*pipe, *from), plan.point(*pipe, *to));
            let _ = write!(
                s,
                r#"<line class="stretch" x1="{:.1}" y1="{:.1}" x2="{:.1}" y2="{:.1}"/>"#,
                a.0, a.1, b.0, b.1
            );
        }
    }
    for p in 0..network.pipes().len() {
        let ((ax, ay), (bx, by)) = plan.ends(p);
        let class = pipe_class.get(p).copied().unwrap_or("q0");
        let tip = pipe_tip.get(p).map_or(String::new(), |t| esc(t));
        let bends = drawing.bends.get(p).map_or(&[][..], Vec::as_slice);
        if bends.is_empty() {
            let _ = write!(
                s,
                r#"<line class="pipe {class}" x1="{ax:.1}" y1="{ay:.1}" x2="{bx:.1}" y2="{by:.1}"/><line class="pipe-hit" data-tip="{tip}" x1="{ax:.1}" y1="{ay:.1}" x2="{bx:.1}" y2="{by:.1}"/>"#
            );
        } else {
            let mut points = format!("{ax:.1},{ay:.1}");
            for &b in bends {
                let (x, y) = plan.at(b);
                let _ = write!(points, " {x:.1},{y:.1}");
            }
            let _ = write!(points, " {bx:.1},{by:.1}");
            let _ = write!(
                s,
                r#"<polyline class="pipe {class}" points="{points}"/><polyline class="pipe-hit" data-tip="{tip}" points="{points}"/>"#
            );
        }
    }
    let taken = node_marks(&plan, markers, &mut s);
    point_marks(&plan, markers, &taken, &mut s);
    s.push_str("</svg>");
    s
}

/// A label's box on the map (left, top, right, bottom), roughly: 12 px
/// type, about 6.4 px per character.
type LabelBox = (f64, f64, f64, f64);

fn label_box(x: f64, baseline: f64, text: &str, end_anchored: bool) -> LabelBox {
    let w = 6.4 * to_f64(text.chars().count());
    let left = if end_anchored { x - w } else { x };
    (left, baseline - 12.0, left + w, baseline + 3.0)
}

fn overlaps(a: &LabelBox, b: &LabelBox) -> bool {
    a.0 < b.2 && b.0 < a.2 && a.1 < b.3 && b.1 < a.3
}

/// Nodes (reservoirs, loggers, candidate places) and their names; returns
/// where the names went, so point labels can keep clear of them.
fn node_marks(plan: &Plan<'_>, markers: &[Marker], s: &mut String) -> Vec<LabelBox> {
    let mut taken = Vec::new();
    let has = |n: usize, logger: bool| {
        markers.iter().any(|m| match m {
            Marker::Logger(i) => logger && *i == n,
            Marker::Candidate(i) => !logger && *i == n,
            _ => false,
        })
    };
    for (n, &pos) in plan.drawing.positions.iter().enumerate() {
        let (x, y) = plan.at(pos);
        let name = plan.drawing.node_names.get(n).map_or("", String::as_str);
        let reservoir = matches!(
            plan.network.nodes()[n],
            jk_hydraulics::Node::Reservoir { .. }
        );
        if reservoir {
            let _ = write!(
                s,
                r#"<path class="reservoir" d="M{:.1},{:.1} L{:.1},{:.1} L{:.1},{:.1} Z"/>"#,
                x,
                y - 8.0,
                x + 8.0,
                y + 6.0,
                x - 8.0,
                y + 6.0
            );
        } else if has(n, true) {
            let _ = write!(
                s,
                r#"<circle class="logger" cx="{x:.1}" cy="{y:.1}" r="7"/>"#
            );
        } else if has(n, false) {
            let _ = write!(
                s,
                r#"<circle class="candidate" cx="{x:.1}" cy="{y:.1}" r="4"/>"#
            );
        }
        // On a big network, names only where something is marked.
        if plan.drawing.positions.len() > 40 && !(reservoir || has(n, true) || has(n, false)) {
            continue;
        }
        let class = if has(n, true) {
            "label strong"
        } else {
            "label"
        };
        // Names near the right edge go to the left of their node.
        let (lx, anchor) = if x > plan.width - 160.0 {
            (x - 10.0, "end")
        } else {
            (x + 10.0, "start")
        };
        taken.push(label_box(lx, y - 10.0, name, anchor == "end"));
        let _ = write!(
            s,
            r#"<text class="{class}" x="{lx:.1}" y="{:.1}" text-anchor="{anchor}">{}</text>"#,
            y - 10.0,
            esc(name)
        );
    }
    taken
}

/// Points on pipes: the estimate (a ring) and the truth (a cross), each
/// labelled - the estimate above, the truth below, since when the engine
/// is right the two marks coincide.
fn point_marks(plan: &Plan<'_>, markers: &[Marker], taken: &[LabelBox], s: &mut String) {
    let mut taken = taken.to_vec();
    for m in markers {
        let Marker::Point {
            pipe,
            distance,
            label,
            truth,
        } = m
        else {
            continue;
        };
        let (x, y) = plan.point(*pipe, *distance);
        if *truth {
            let _ = write!(
                s,
                r#"<path class="truth" d="M{:.1},{:.1} L{:.1},{:.1} M{:.1},{:.1} L{:.1},{:.1}"/>"#,
                x - 6.0,
                y - 6.0,
                x + 6.0,
                y + 6.0,
                x - 6.0,
                y + 6.0,
                x + 6.0,
                y - 6.0
            );
        } else {
            let _ = write!(
                s,
                r#"<circle class="likely" cx="{x:.1}" cy="{y:.1}" r="6"/>"#
            );
        }
        // Estimate above, truth below; step further away until clear of
        // every name already placed.
        let (first, step) = if *truth { (26.0, 18.0) } else { (-14.0, -18.0) };
        let baseline = (0..5)
            .map(|k| y + first + step * to_f64(k))
            .find(|b| {
                let bx = label_box(x + 12.0, *b, label, false);
                taken.iter().all(|t| !overlaps(&bx, t))
            })
            .unwrap_or(y + first);
        taken.push(label_box(x + 12.0, baseline, label, false));
        let _ = write!(
            s,
            r#"<text class="label point-label" x="{:.1}" y="{baseline:.1}">{}</text>"#,
            x + 12.0,
            esc(label)
        );
    }
}

/// A column chart of one series, value on every cap (few columns).
pub fn columns(items: &[(String, f64)], unit: &str, width: f64, height: f64) -> String {
    let (left, right, top, bottom) = (48.0, 12.0, 22.0, 30.0);
    let max = items.iter().map(|(_, v)| *v).fold(0.0, f64::max);
    let (y_ticks, y_step) = ticks(0.0, max * 1.1, 4);
    let y1 = y_ticks.last().copied().unwrap_or(max).max(max * 1.1);
    let sy = Scale::new((0.0, y1), (height - bottom, top));
    let band = (width - left - right) / to_f64(items.len().max(1));
    let bar = (band * 0.5).min(24.0);
    let mut s = format!(r#"<svg class="chart columns" viewBox="0 0 {width} {height}" role="img">"#);
    for v in &y_ticks {
        let y = sy.at(*v);
        let _ = write!(
            s,
            r#"<line class="grid" x1="{left}" x2="{}" y1="{y:.1}" y2="{y:.1}"/><text class="tick" x="{}" y="{:.1}" text-anchor="end">{}</text>"#,
            width - right,
            left - 6.0,
            y + 4.0,
            tick_label(*v, y_step)
        );
    }
    for (i, (label, v)) in items.iter().enumerate() {
        let cx = left + band * (to_f64(i) + 0.5);
        let (x, w) = (cx - bar / 2.0, bar);
        let (y, base) = (sy.at(*v), sy.at(0.0));
        let r = 4.0_f64.min((base - y) / 2.0).max(0.0);
        let _ = write!(
            s,
            r#"<path class="bar" data-tip="{} loggers|{v:.0} {unit}" d="M{x:.1},{base:.1} L{x:.1},{:.1} Q{x:.1},{y:.1} {:.1},{y:.1} L{:.1},{y:.1} Q{:.1},{y:.1} {:.1},{:.1} L{:.1},{base:.1} Z"/>"#,
            esc(label),
            y + r,
            x + r,
            x + w - r,
            x + w,
            x + w,
            y + r,
            x + w
        );
        let _ = write!(
            s,
            r#"<text class="value" x="{cx:.1}" y="{:.1}" text-anchor="middle">{v:.0} {}</text><text class="tick" x="{cx:.1}" y="{:.1}" text-anchor="middle">{}</text>"#,
            y - 6.0,
            esc(unit),
            height - 10.0,
            esc(label)
        );
    }
    let _ = write!(
        s,
        r#"<line class="axis" x1="{left}" x2="{}" y1="{:.1}" y2="{:.1}"/></svg>"#,
        width - right,
        sy.at(0.0),
        sy.at(0.0)
    );
    s
}
