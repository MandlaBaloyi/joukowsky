//! Pipe health: what a pump trip does to a rising main - the loggers'
//! records, the alert, and the fatigue loading each logger's pipes took.

use std::fmt::Write as _;

use super::{ChartRecord, PRESSURE, badge, class_name};
use crate::view::charts::columns;
use crate::view::health_model::{EXPONENT, HealthModel, REFERENCE, TRIPS_PER_DAY};
use crate::view::shell::Page;
use crate::view::svg::esc;

/// The charts: the three loggers through the trip, with the pipes' rating.
pub fn records(m: &HealthModel) -> Vec<ChartRecord<'_>> {
    m.loggers
        .iter()
        .map(|l| ChartRecord {
            name: l.name.clone(),
            interval: 1.0 / crate::view::model::RATE,
            runs: vec![(l.start, l.samples.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: PRESSURE,
            marks: vec![(3.0, "power lost".into())],
            lines: vec![(m.rating, "PN10 rating".into())],
        })
        .collect()
}

#[allow(clippy::too_many_lines)] // one page, top to bottom
pub fn page(m: &HealthModel) -> Page {
    let per_year = TRIPS_PER_DAY * 365.0;
    let worst = m
        .loggers
        .iter()
        .max_by(|a, b| a.equivalent().total_cmp(&b.equivalent()));
    let highest = m
        .loggers
        .iter()
        .flat_map(|l| l.samples.iter().copied())
        .fold(f64::MIN, f64::max);
    let mut s = String::new();
    if let Some(w) = worst {
        let _ = write!(
            s,
            r#"<div class="stats">
<div class="stat"><div class="label">Hardest-hit logger</div><div class="value">{}</div><div class="detail">{:.1} swings of {REFERENCE:.0} m - equivalent - per trip</div></div>
<div class="stat"><div class="label">A year of load-shedding</div><div class="value">{:.0}</div><div class="detail">equivalent {REFERENCE:.0} m swings there, at {TRIPS_PER_DAY:.0} trips a day</div></div>
<div class="stat"><div class="label">Highest pressure</div><div class="value">{highest:.1} m</div><div class="detail">the pipes are rated for {:.1} m (PN10)</div></div>
</div>"#,
            esc(&w.name),
            w.equivalent(),
            w.equivalent() * per_year,
            m.rating
        );
    }
    let duration = m.duration;
    let _ = write!(
        s,
        r#"<div class="card"><h2>One pump trip, three loggers</h2><p class="note">A pump lifts 100 l/s from its sump up a 2 km DN300 main into a reservoir. Load-shedding cuts its power at 3 s. The pump runs down, the water column in the main slows and swings back: the pressure falls at the pump, then rings up and down along the main for long after. Every swing is a load cycle on the pipe wall. The dashed line is the pipes' rating. Hover to read; drag to zoom.</p>"#
    );
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="health-traces"><button type="button" class="chip" data-range="whole,0,{duration}" aria-pressed="true">All {duration:.0} s</button><span class="hint">showing 0 - {duration:.0} s - drag across a chart to zoom</span></div><div class="js-traces" id="health-traces" data-set="health" data-unit="s" data-from="0" data-to="{duration}" data-zoomable></div></div>"#
    );
    // Loading, per logger.
    let items: Vec<(String, f64)> = m
        .loggers
        .iter()
        .map(|l| (l.name.clone(), l.equivalent() * per_year))
        .collect();
    s.push_str(r#"<div class="cols"><div class="card"><h2>A year of trips, per logger</h2><p class="note">Equivalent swings of 50 m: every swing the logger saw, weighed by how much a pipe tires from it.</p>"#);
    s.push_str(&columns(&items, "", 520.0, 300.0, "a year"));
    s.push_str(r#"</div><div class="card"><h2>The swings counted</h2><div class="table-wrap"><table><thead><tr><th>Logger</th><th class="num">0.5-5 m</th><th class="num">5-20 m</th><th class="num">20-50 m</th><th class="num">50 m +</th><th class="num">Equivalent 50 m swings</th></tr></thead><tbody>"#);
    for l in &m.loggers {
        let _ = write!(
            s,
            r#"<tr><td>{}</td><td class="num">{:.1}</td><td class="num">{:.1}</td><td class="num">{:.1}</td><td class="num">{:.1}</td><td class="num">{:.2}</td></tr>"#,
            esc(&l.name),
            l.band(0.0, 5.0),
            l.band(5.0, 20.0),
            l.band(20.0, 50.0),
            l.band(50.0, f64::INFINITY),
            l.equivalent()
        );
    }
    s.push_str(r#"</tbody></table></div><p class="note">Counts per trip (half a swing counts 0.5). Swings under 0.5 m are logger noise and not counted.</p></div></div>"#);
    // The alert(s).
    s.push_str(r#"<div class="card"><h2>What the engine said</h2>"#);
    if m.alerts.is_empty() {
        s.push_str("<p>No alert.</p>");
    }
    for a in &m.alerts {
        let _ = write!(
            s,
            r#"<p>{} <b>{}</b> - {:.0}% confident{}</p><ul class="reasons">"#,
            badge(a.level),
            esc(&class_name(a)),
            a.confidence * 100.0,
            if a.revision > 0 {
                format!(" (revision {})", a.revision)
            } else {
                String::new()
            }
        );
        for r in &a.reasons {
            let _ = write!(s, "<li>{}</li>", esc(r));
        }
        s.push_str("</ul>");
    }
    s.push_str("</div>");
    let _ = write!(
        s,
        r#"<div class="card"><h2>How to read this</h2><ul class="reasons"><li>Rainflow counting (ASTM E1049) pairs every rise with the fall that closes it, so a record becomes a list of swings - big ones kept whole even when small ripples interrupt them.</li><li>A pipe tires much faster from big swings than small ones: one swing of twice the size counts like 2<sup>m</sup> small ones. Here m = {EXPONENT:.0} [to be checked per pipe material]. The equivalent number of 50 m swings needs only m - no fatigue data for 40-year-old pipes, which nobody has.</li><li>So this RANKS pipes ("this one takes 3x the loading of that one"); it does not predict when a pipe breaks. Repair records will teach the engine that later.</li><li>The yearly figure assumes {TRIPS_PER_DAY:.0} trips a day, each like this one.</li><li>Counted on what the loggers measured. The engine never counts fatigue from a simulated boiling zone: those models send numerical pulses there (FN-038).</li></ul></div>"#
    );
    Page {
        id: "health",
        title: "Pipe health".into(),
        crumb: "a pump station on load-shedding: one trip, three loggers".into(),
        source: "Simulated data",
        body: s,
    }
}
