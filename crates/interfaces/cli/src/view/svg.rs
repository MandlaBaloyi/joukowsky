//! Small drawing helpers: escaping, scales, tidy axis ticks.

use std::fmt::Write as _;

/// Escapes text for HTML and SVG (names come from site descriptions and
/// data files: never trusted as markup).
pub fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

/// A linear map from data values to pixels.
#[derive(Debug, Clone, Copy)]
pub struct Scale {
    d0: f64,
    d1: f64,
    r0: f64,
    r1: f64,
}

impl Scale {
    pub fn new((d0, d1): (f64, f64), (r0, r1): (f64, f64)) -> Self {
        let d1 = if (d1 - d0).abs() < 1e-12 {
            d0 + 1.0
        } else {
            d1
        };
        Self { d0, d1, r0, r1 }
    }

    pub fn at(&self, v: f64) -> f64 {
        self.r0 + (v - self.d0) / (self.d1 - self.d0) * (self.r1 - self.r0)
    }
}

/// About `count` round tick values covering [min, max] (1, 2, 5 x 10^n
/// steps), and the step.
pub fn ticks(min: f64, max: f64, count: usize) -> (Vec<f64>, f64) {
    let span = (max - min).abs().max(1e-9);
    let raw = span / to_f64(count.max(1));
    let magnitude = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0]
        .iter()
        .map(|m| m * magnitude)
        .find(|s| *s >= raw)
        .unwrap_or(10.0 * magnitude);
    let first = (min / step).ceil() * step;
    let mut out = Vec::new();
    let mut v = first;
    while v <= max + step * 1e-9 {
        out.push(if v.abs() < step * 1e-9 { 0.0 } else { v });
        v += step;
    }
    (out, step)
}

/// A tick label with only the decimals the step needs.
pub fn tick_label(v: f64, step: f64) -> String {
    let decimals = if step >= 1.0 {
        0
    } else {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small, positive
        let d = (-step.log10()).ceil() as usize;
        d
    };
    format!("{v:.decimals$}")
}

/// One of the 8 sequential colour classes (`q1`..`q8`) for `t` in [0, 1];
/// `q0` (neutral) for nothing.
pub fn seq_class(t: f64) -> &'static str {
    const CLASSES: [&str; 9] = ["q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8"];
    if !(t.is_finite() && t > 0.0) {
        return CLASSES[0];
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // 1..=8
    let i = ((t.min(1.0) * 8.0).ceil() as usize).clamp(1, 8);
    CLASSES[i]
}

/// Numbers as a compact JSON array (3 significant decimals).
pub fn json_numbers(values: &[f64], decimals: usize) -> String {
    let mut out = String::from("[");
    for (i, v) in values.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        let _ = write!(out, "{v:.decimals$}");
    }
    out.push(']');
    out
}

#[allow(clippy::cast_precision_loss)] // small counts
pub fn to_f64(i: usize) -> f64 {
    i as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_markup() {
        assert_eq!(esc("a<b & \"c\""), "a&lt;b &amp; &quot;c&quot;");
    }

    #[test]
    fn ticks_are_round() {
        let (t, step) = ticks(0.0, 40.0, 8);
        assert!((step - 5.0).abs() < 1e-12);
        assert_eq!(t.len(), 9);
        let (t, _) = ticks(51.3, 58.9, 4);
        assert_eq!(t, vec![52.0, 54.0, 56.0, 58.0]);
    }

    #[test]
    fn sequential_classes_cover_the_range() {
        assert_eq!(seq_class(0.0), "q0");
        assert_eq!(seq_class(0.01), "q1");
        assert_eq!(seq_class(1.0), "q8");
    }
}
