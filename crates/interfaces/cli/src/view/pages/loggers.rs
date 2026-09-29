//! Logger records: every logger's pressure, zoomable, one crosshair for
//! all of them.

use std::fmt::Write as _;

use crate::view::model::EventModel;
use crate::view::shell::Page;

pub fn page(event: &EventModel) -> Page {
    let (t0, t1) = event.span();
    let whole = format!("whole,{t0:.3},{t1:.3}");
    let mut s = String::from(
        r#"<div class="card"><p class="note">Pressure head at each logger (metres), on one time axis (seconds). A line marks each front the detector picked. Shaded: the logger was not watching at full sensitivity - learning its noise, deaf just after a front, or listening through the ringing that follows - so its silence there does not mean "nothing happened". Hover to read every logger at once; drag across a chart to zoom.</p>"#,
    );
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="records"><button type="button" class="chip" data-range="{whole}" aria-pressed="true">Whole record</button>"#
    );
    for i in &event.issued {
        let t = i.alert.time.value();
        let _ = write!(
            s,
            r#"<button type="button" class="chip" data-range="alert{id},{:.3},{:.3}" aria-pressed="false">Around alert #{id}</button>"#,
            (t - 1.0).max(t0),
            (t + 4.0).min(t1),
            id = i.alert.id
        );
    }
    let _ = write!(
        s,
        r#"<span class="hint">showing {t0:.2} - {t1:.2} s - drag across a chart to zoom</span></div><div class="js-traces" id="records" data-from="{t0:.3}" data-to="{t1:.3}" data-zoomable></div><div class="legend"><span class="key"><span class="swatch"></span>not watching at full sensitivity</span><span class="key"><span class="mark">|</span>front picked by the detector</span></div></div>"#
    );
    Page {
        id: "loggers",
        title: "Logger records".into(),
        crumb: format!("{} loggers, 128 samples a second", event.records.len()),
        body: s,
    }
}
