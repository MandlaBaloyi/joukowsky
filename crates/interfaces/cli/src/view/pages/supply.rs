//! Tanks & supply: an estate's tank running dry during a bulk-supply
//! outage - the extended-period run and its supply warnings.

use std::fmt::Write as _;

use super::ChartRecord;
use crate::view::shell::Page;
use crate::view::supply_model::SupplyModel;
use crate::view::svg::esc;

/// The charts: the tank's level and the head at the far end.
pub fn records(m: &SupplyModel) -> Vec<ChartRecord<'_>> {
    let dt = m.hours.get(1).map_or(1.0, |t| t - m.hours[0]);
    let marks: Vec<(f64, String)> = m.events.clone();
    vec![
        ChartRecord {
            name: "Tank level".into(),
            interval: dt,
            runs: vec![(0.0, m.level.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("h", "m"),
            marks: marks.clone(),
            lines: vec![
                (m.ceiling, "overflow".into()),
                (m.floor, "floor: no supply below".into()),
            ],
        },
        ChartRecord {
            name: "Head at the far end of the estate".into(),
            interval: dt,
            runs: vec![(0.0, m.far_head.as_slice())],
            fronts: Vec::new(),
            impaired: Vec::new(),
            units: ("h", "m"),
            marks,
            lines: Vec::new(),
        },
    ]
}

pub fn page(m: &SupplyModel) -> Page {
    let at = |what: &str| {
        m.events
            .iter()
            .find(|(_, e)| e == what)
            .map_or("-".to_string(), |(t, _)| format!("{t:.1} h"))
    };
    let hours_left = m.volume / (m.demand / 1000.0 * 3600.0);
    let end = m.duration;
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Water above the floor at the start</div><div class="value">{:.0} m&sup3;</div><div class="detail">about {hours_left:.0} h at the average {:.0} l/s</div></div>
<div class="stat"><div class="label">Tank empty</div><div class="value">{}</div><div class="detail">after the outage began</div></div>
<div class="stat"><div class="label">Supply lost</div><div class="value">{}</div><div class="detail">no source reaches the estate</div></div>
</div>"#,
        m.volume,
        m.demand,
        esc(&at("tank empty")),
        esc(&at("supply lost"))
    );
    s.push_str(r#"<div class="card"><h2>The outage, hour by hour</h2><p class="note">The municipal feed is off; the estate's tank (400 m&sup2;) supplies three blocks through 2.1 km of main. Every 5 minutes the engine works out the flows with that hour's demand - low at night, peaks at 07:00 and 19:00 - and moves the tank by what it lost. The level falls fastest at the peaks. At its floor the tank stops supplying and the run reports the supply lost. Hover to read both charts at once; drag to zoom.</p>"#);
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="supply-traces"><button type="button" class="chip" data-range="whole,0,{end:.2}" aria-pressed="true">Both days</button><span class="note">the line stops where supply is lost</span><span class="hint">showing 0 - {end:.2} h - drag across a chart to zoom</span></div><div class="js-traces" id="supply-traces" data-set="supply" data-unit="h" data-from="0" data-to="{end:.2}" data-zoomable></div></div>"#
    );
    s.push_str(r#"<div class="card"><h2>What the warning is for</h2><ul class="reasons"><li>Hours ahead, the engine knows when the tank will run dry: time to cut non-essential use, arrange water tankers, warn residents.</li><li>When supply is lost the pipes drain and fill with air. When supply RETURNS, the refilling surge is among the most violent transients in South African networks - open valves slowly and watch the loggers.</li><li>Demand here follows a fixed daily pattern; real runs will use the site's metered demand.</li></ul></div>"#);
    Page {
        id: "supply",
        title: "Tanks & supply".into(),
        crumb: "an estate's tank through a bulk-supply outage".into(),
        source: "Simulated data",
        body: s,
    }
}
