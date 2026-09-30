//! Tanks & supply: an estate's tank running dry during a bulk-supply
//! outage - the extended-period run and its supply warnings.

use std::fmt::Write as _;

use super::ChartRecord;
use crate::view::shell::Page;
use crate::view::supply_model::{PumpDayModel, SupplyModel};
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

/// The pump day's charts: the tank level with the pump's switch levels, and
/// the pump running; load-shedding slots shaded on both.
pub fn pump_records(p: &PumpDayModel) -> Vec<ChartRecord<'_>> {
    let dt = p.hours.get(1).map_or(1.0, |t| t - p.hours[0]);
    vec![
        ChartRecord {
            name: "Tank level".into(),
            interval: dt,
            runs: vec![(0.0, p.level.as_slice())],
            fronts: Vec::new(),
            impaired: p.cuts.clone(),
            units: ("h", "m"),
            marks: p.events.clone(),
            lines: vec![
                (p.off_level, "pump stops".into()),
                (p.on_level, "pump starts".into()),
                (p.floor, "floor".into()),
            ],
        },
        ChartRecord {
            name: "Pump running (1 = on)".into(),
            interval: dt,
            runs: vec![(0.0, p.running.as_slice())],
            fronts: Vec::new(),
            impaired: p.cuts.clone(),
            units: ("h", ""),
            marks: Vec::new(),
            lines: Vec::new(),
        },
    ]
}

/// The pump day's card.
fn pump_card(s: &mut String, p: &PumpDayModel) {
    let waited = p
        .waits
        .iter()
        .map(|(asked, started)| {
            format!(
                "asked at {} and started at {} ({:.1} h later)",
                clock(*asked),
                clock(*started),
                started - asked
            )
        })
        .collect::<Vec<_>>();
    let end = p.duration;
    let _ = write!(
        s,
        r#"<div class="card"><h2>A pump and load-shedding, two days</h2><p class="note">A pump lifts water from a sump up 600 m of main into the estate's 150 m&sup2; tank. It starts when the tank falls to {:.1} m and stops at {:.1} m. Load-shedding (shaded) takes the power 04:00-06:30 and 18:00-20:30: no pump runs then, and when the power returns the pump does what its level control last asked. The engine lands exactly on every switch - the level never overshoots the switch levels - and each start and stop is a sudden change of flow: a pressure wave from a known source, at a known time.</p>
<div class="stats">
<div class="stat"><div class="label">Pump ran</div><div class="value">{:.1} h</div><div class="detail">of {end:.0} h</div></div>
<div class="stat"><div class="label">Lowest level</div><div class="value">{:.2} m</div><div class="detail">at {}, {:.2} m above the floor</div></div>
<div class="stat"><div class="label">Starts delayed by load-shedding</div><div class="value">{}</div><div class="detail">{}</div></div>
</div>"#,
        p.on_level,
        p.off_level,
        p.run_hours,
        p.lowest.1,
        clock(p.lowest.0),
        p.lowest.1 - p.floor,
        p.waits.len(),
        esc(&if waited.is_empty() {
            "none: the tank had water enough".to_string()
        } else {
            waited.join("; ")
        })
    );
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="pumpday-traces"><button type="button" class="chip" data-range="whole,0,{end:.2}" aria-pressed="true">Both days</button><button type="button" class="chip" data-range="evening,16,22">First evening</button><span class="hint">showing 0 - {end:.2} h - drag across a chart to zoom</span></div><div class="js-traces" id="pumpday-traces" data-set="pumpday" data-unit="h" data-from="0" data-to="{end:.2}" data-zoomable></div></div>"#
    );
}

/// Hours since the first midnight as a day and clock time ("day 2 18:40").
fn clock(hours: f64) -> String {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small, >= 0
    let minutes = (hours * 60.0).round() as u64;
    format!(
        "day {} {:02}:{:02}",
        minutes / 1440 + 1,
        minutes % 1440 / 60,
        minutes % 60
    )
}

pub fn page(m: &SupplyModel, p: &PumpDayModel) -> Page {
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
    pump_card(&mut s, p);
    s.push_str(r#"<div class="card"><h2>What the warning is for</h2><ul class="reasons"><li>Hours ahead, the engine knows when the tank will run dry: time to cut non-essential use, arrange water tankers, warn residents.</li><li>When supply is lost the pipes drain and fill with air. When supply RETURNS, the refilling surge is among the most violent transients in South African networks - open valves slowly and watch the loggers.</li><li>Demand here follows a fixed daily pattern; real runs will use the site's metered demand.</li></ul></div>"#);
    Page {
        id: "supply",
        title: "Tanks & supply".into(),
        crumb: "an estate's tank through a supply outage, and a pump through load-shedding".into(),
        source: "Simulated data",
        body: s,
    }
}
