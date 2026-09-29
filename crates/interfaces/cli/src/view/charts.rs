//! The three kinds of picture on the page, as inline SVG.
//!
//! Colours are never written here: every mark carries a CSS class whose
//! colour the page defines once per theme (light and dark), so the charts
//! follow the reader's setting. Text is escaped; hover details ride on
//! `data-*` attributes and are shown by the page's script with
//! `textContent` only.

use std::fmt::Write as _;

use jk_hydraulics::Network;

use super::svg::{Scale, esc, json_numbers, tick_label, ticks, to_f64};
use crate::sim::sites::Drawing;

/// One logger's pressure record, for [`trace`].
pub struct TraceData<'a> {
    /// Sample times [s].
    pub times: &'a [f64],
    /// Heads [m].
    pub heads: &'a [f64],
    /// Detected fronts: arrival times [s].
    pub fronts: &'a [f64],
    /// Stretches when the logger was not watching at full sensitivity [s].
    pub impaired: &'a [(f64, f64)],
}

/// A pressure trace over `x_range` seconds: the line, the detected fronts
/// (labelled), the impaired stretches (shaded), a hover crosshair.
pub fn trace(data: &TraceData<'_>, x_range: (f64, f64), width: f64, height: f64) -> String {
    let (left, right, top, bottom) = (52.0, 12.0, 22.0, 26.0);
    let (lo, hi) = data
        .heads
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &h| {
            (a.min(h), b.max(h))
        });
    let pad = ((hi - lo) * 0.08).max(0.05);
    let (y_ticks, y_step) = ticks(lo - pad, hi + pad, 5);
    let y0 = y_ticks.first().copied().unwrap_or(lo).min(lo - pad);
    let y1 = y_ticks.last().copied().unwrap_or(hi).max(hi + pad);
    let sx = Scale::new(x_range, (left, width - right));
    let sy = Scale::new((y0, y1), (height - bottom, top));
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<svg class="chart trace" viewBox="0 0 {width} {height}" role="img" data-sx="{left},{},{},{}" "#,
        width - right,
        x_range.0,
        x_range.1
    );

    // Hover data: a decimated copy of the record (min and max per pixel).
    let (dt, dh) = decimate(data.times, data.heads, &sx);
    let _ = write!(
        s,
        r#"data-t="{}" data-h="{}">"#,
        json_numbers(&dt, 3),
        json_numbers(&dh, 3)
    );

    // Impaired stretches first, under everything.
    for &(a, b) in data.impaired {
        let (xa, xb) = (sx.at(a.max(x_range.0)), sx.at(b.min(x_range.1)));
        if xb > xa {
            let _ = write!(
                s,
                r#"<rect class="impaired" x="{xa:.1}" y="{top}" width="{:.1}" height="{:.1}"/>"#,
                xb - xa,
                height - bottom - top
            );
        }
    }
    // Grid and axes.
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
    let (x_ticks, x_step) = ticks(x_range.0, x_range.1, 8);
    for v in &x_ticks {
        let x = sx.at(*v);
        let _ = write!(
            s,
            r#"<text class="tick" x="{x:.1}" y="{:.1}" text-anchor="middle">{}</text>"#,
            height - 8.0,
            tick_label(*v, x_step)
        );
    }
    let _ = write!(
        s,
        r#"<line class="axis" x1="{left}" x2="{}" y1="{:.1}" y2="{:.1}"/>"#,
        width - right,
        height - bottom,
        height - bottom
    );
    // The record.
    let mut path = String::new();
    for (i, (t, h)) in dt.iter().zip(&dh).enumerate() {
        let _ = write!(
            path,
            "{}{:.1},{:.1}",
            if i == 0 { "M" } else { "L" },
            sx.at(*t),
            sy.at(*h)
        );
    }
    let _ = write!(s, r#"<path class="series" d="{path}"/>"#);
    // Detected fronts, labelled.
    for &f in data.fronts {
        let x = sx.at(f);
        let _ = write!(
            s,
            r#"<line class="front" x1="{x:.1}" x2="{x:.1}" y1="{top}" y2="{:.1}"/><text class="front-label" x="{:.1}" y="{:.1}">front {f:.3} s</text>"#,
            height - bottom,
            x + 4.0,
            top - 6.0
        );
    }
    // Crosshair (moved by the page script).
    let _ = write!(
        s,
        r#"<line class="crosshair" x1="0" x2="0" y1="{top}" y2="{:.1}" visibility="hidden"/><rect class="hit" x="{left}" y="{top}" width="{:.1}" height="{:.1}"/></svg>"#,
        height - bottom,
        width - right - left,
        height - bottom - top
    );
    s
}

/// Keeps the smallest and largest value in every pixel column, in time
/// order: the drawn line looks exactly like the full record at a fraction
/// of the size.
fn decimate(times: &[f64], heads: &[f64], sx: &Scale) -> (Vec<f64>, Vec<f64>) {
    let (mut t_out, mut h_out) = (Vec::new(), Vec::new());
    let mut i = 0;
    while i < times.len() {
        #[allow(clippy::cast_possible_truncation)] // pixel columns
        let column = sx.at(times[i]).floor() as i64;
        let mut j = i;
        let (mut lo, mut hi) = (i, i);
        #[allow(clippy::cast_possible_truncation)]
        while j < times.len() && sx.at(times[j]).floor() as i64 == column {
            if heads[j] < heads[lo] {
                lo = j;
            }
            if heads[j] > heads[hi] {
                hi = j;
            }
            j += 1;
        }
        for k in if lo <= hi { [lo, hi] } else { [hi, lo] } {
            if t_out.last() != Some(&times[k]) {
                t_out.push(times[k]);
                h_out.push(heads[k]);
            }
        }
        i = j;
    }
    (t_out, h_out)
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
        for &(x, y) in &drawing.positions {
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
            k,
            width,
            height,
        }
    }

    fn at(&self, (x, y): (f64, f64)) -> (f64, f64) {
        (
            self.pad + (x - self.x0) * self.k,
            self.pad + (y - self.y0) * self.k,
        )
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
        r#"<svg class="chart map" viewBox="0 0 {width} {:.0}" role="img">"#,
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
        let _ = write!(
            s,
            r#"<line class="pipe {class}" x1="{ax:.1}" y1="{ay:.1}" x2="{bx:.1}" y2="{by:.1}"/><line class="pipe-hit" data-tip="{tip}" x1="{ax:.1}" y1="{ay:.1}" x2="{bx:.1}" y2="{by:.1}"/>"#
        );
    }
    node_marks(&plan, markers, &mut s);
    point_marks(&plan, markers, &mut s);
    s.push_str("</svg>");
    s
}

/// Nodes (reservoirs, loggers, candidate places) and their names.
fn node_marks(plan: &Plan<'_>, markers: &[Marker], s: &mut String) {
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
        let _ = write!(
            s,
            r#"<text class="{class}" x="{lx:.1}" y="{:.1}" text-anchor="{anchor}">{}</text>"#,
            y - 10.0,
            esc(name)
        );
    }
}

/// Points on pipes: the estimate (a ring) and the truth (a cross), each
/// labelled - the estimate above, the truth below, since when the engine
/// is right the two marks coincide.
fn point_marks(plan: &Plan<'_>, markers: &[Marker], s: &mut String) {
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
        let _ = write!(
            s,
            r#"<text class="label point-label" x="{:.1}" y="{:.1}">{}</text>"#,
            x + 12.0,
            y + if *truth { 26.0 } else { -14.0 },
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
