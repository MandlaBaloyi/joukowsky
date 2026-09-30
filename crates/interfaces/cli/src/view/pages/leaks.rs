//! Night watch: a metered zone's night flows over 90 days, and the slow
//! leak the water balance catches - no wave needed.

use std::fmt::Write as _;

use super::ChartRecord;
use crate::view::shell::Page;
use crate::view::supply_model::NightModel;
use crate::view::svg::esc;

/// The charts: night flows with the normal level, and the watch's CUSUM.
pub fn records(m: &NightModel) -> Vec<ChartRecord<'_>> {
    let base = m.watch.baseline.value() * 1000.0;
    let mut marks = vec![(
        #[allow(clippy::cast_precision_loss)] // small
        (m.leak_night as f64),
        "leak begins (truth)".to_string(),
    )];
    for a in &m.watch.alarms {
        marks.push((m.night_of(a.raised), "ALARM".into()));
    }
    vec![
        ChartRecord {
            name: "Night flow, 02:00-04:00".into(),
            interval: 1.0,
            runs: vec![(0.0, m.nights.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("day", "l/s"),
            marks: marks.clone(),
            lines: vec![
                (base, "normal nights".into()),
                (
                    base + m.watch.smallest_in_a_week.value() * 1000.0,
                    "a leak this big is caught within a week".into(),
                ),
            ],
        },
        ChartRecord {
            name: "Evidence of a rise (CUSUM, in spreads)".into(),
            interval: 1.0,
            runs: vec![(0.0, m.watch.cusum.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("day", ""),
            marks,
            lines: vec![(6.0, "alarm level".into())],
        },
    ]
}

pub fn page(m: &NightModel) -> Page {
    let alarm = m.watch.alarms.first();
    let (first, raised, size) = alarm.map_or(("-".into(), "-".into(), "-".into()), |a| {
        (
            format!("night {:.0}", m.night_of(a.first_night)),
            format!("night {:.0}", m.night_of(a.raised)),
            format!(
                "{:.2} +/- {:.2} l/s",
                a.size.value() * 1000.0,
                a.uncertainty.value() * 1000.0
            ),
        )
    });
    let days = m.nights.len();
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Normal night flow</div><div class="value">{:.2} l/s</div><div class="detail">nights vary by {:.2} l/s</div></div>
<div class="stat"><div class="label">Smallest leak caught in a week</div><div class="value">{:.2} l/s</div><div class="detail">set by how much nights vary</div></div>
<div class="stat"><div class="label">Leak found</div><div class="value">{}</div><div class="detail">began {} (truth: night {}, {:.1} l/s); raised {}</div></div>
</div>"#,
        m.watch.baseline.value() * 1000.0,
        m.watch.spread.value() * 1000.0,
        m.watch.smallest_in_a_week.value() * 1000.0,
        esc(&size),
        esc(&first),
        m.leak_night,
        m.leak,
        esc(&raised)
    );
    let missing = m
        .missing
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join(", ");
    let _ = write!(
        s,
        r#"<div class="card"><h2>Night after night</h2><p class="note">The zone's meters give its balance: water in, minus water passed on. Between 02:00 and 04:00 almost nobody uses water, so the night flow is mostly leakage. The first 28 nights set what "normal" is; from then on the watch adds up how far each night sits above normal - the second chart - and raises the alarm when the evidence passes the level. Gaps are nights a logger's battery lost (nights {missing}): left empty, never guessed. Hover to read both charts; drag to zoom.</p>"#
    );
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="night-traces"><button type="button" class="chip" data-range="whole,0,{days}" aria-pressed="true">All {days} days</button><span class="hint">showing 0 - {days} day - drag across a chart to zoom</span></div><div class="js-traces" id="night-traces" data-set="night" data-unit="day" data-from="0" data-to="{days}" data-zoomable></div></div>"#
    );
    s.push_str(r#"<div class="card"><h2>Why this matters</h2><ul class="reasons"><li>A leak growing quietly sends no pressure wave - the transient engine cannot see it. The water balance can.</li><li>A meter's calibration error shifts every night alike, so it cannot hide a change; it only blurs the leak's size - included in the uncertainty.</li><li>The smallest leak caught depends on how much nights vary, not on how big the pipe is: on a bulk main this finds leaks far smaller than any wave would reveal.</li></ul></div>"#);
    Page {
        id: "leaks",
        title: "Night watch".into(),
        crumb: format!("a metered zone, {days} days of 15-minute readings"),
        source: "Simulated data",
        body: s,
    }
}
