//! Recordings: logger files read by the logger reader - real pressure data
//! when real files are given.
//!
//! For each file: what the reader found and decided (its notes), and every
//! pressure channel as a zoomable chart on one shared time axis (seconds
//! from the earliest sample of all files). The detector runs on each
//! continuous stretch and its fronts are marked. No network is known for
//! these files, so nothing is located and the detector uses its default
//! settings - the charts show what the engine SEES, not an alert.

use std::fmt::Write as _;

use jk_detection::{Detector, DetectorConfig, Series};
use jk_files::logger::Recording;
use jk_kernel::Time;

use super::ChartRecord;
use crate::view::shell::Page;
use crate::view::svg::{esc, to_f64};

/// A file read, and its name.
pub struct FileRecording {
    pub name: String,
    pub recording: Recording,
}

/// The earliest sample of all files [s] - the charts' zero.
fn origin(files: &[FileRecording]) -> f64 {
    files
        .iter()
        .flat_map(|f| f.recording.runs.iter().map(|r| r.start.value()))
        .fold(f64::INFINITY, f64::min)
}

/// Every channel of every file as chart records, with the detector's
/// fronts and impaired stretches (times from `origin`).
pub fn chart_records(files: &[FileRecording]) -> Vec<ChartRecord<'_>> {
    let zero = origin(files);
    let detector = Detector::new(DetectorConfig::default()).ok();
    let mut out = Vec::new();
    for f in files {
        let r = &f.recording;
        let dt = r.interval.value();
        for (ch, c) in r.channels.iter().enumerate() {
            let mut fronts = Vec::new();
            let mut impaired = Vec::new();
            for run in &r.runs {
                let heads = &run.heads[ch];
                // Continuous stretches of valid samples, as the live engine
                // sees them (an invalid sample is a gap).
                let mut i = 0;
                while i < heads.len() {
                    let len = heads[i..].iter().take_while(|h| h.is_finite()).count();
                    if len == 0 {
                        i += 1;
                        continue;
                    }
                    let start = run.start.value() + dt * to_f64(i);
                    let series = Series {
                        samples: &heads[i..i + len],
                        start: Time::new(start - zero),
                        interval: r.interval,
                    };
                    match detector.as_ref().map(|d| d.run(&series)) {
                        Some(Ok(d)) => {
                            fronts.extend(d.events.iter().map(|e| e.arrival.value()));
                            impaired
                                .extend(d.impaired.iter().map(|s| (s.from.value(), s.to.value())));
                        }
                        _ => impaired.push((start - zero, start - zero + dt * to_f64(len))),
                    }
                    i += len;
                }
            }
            out.push(ChartRecord {
                name: format!("{} - {}", f.name, c.name),
                interval: dt,
                runs: r
                    .runs
                    .iter()
                    .map(|run| (run.start.value() - zero, run.heads[ch].as_slice()))
                    .collect(),
                fronts,
                impaired,
                units: super::PRESSURE,
                marks: Vec::new(),
                lines: Vec::new(),
            });
        }
    }
    out
}

pub fn page(files: &[FileRecording]) -> Page {
    let zero = origin(files);
    let end = files
        .iter()
        .flat_map(|f| {
            f.recording.runs.iter().map(|r| {
                r.start.value()
                    + f.recording.interval.value() * to_f64(r.heads.first().map_or(0, Vec::len))
            })
        })
        .fold(zero, f64::max);
    let channels: usize = files.iter().map(|f| f.recording.channels.len()).sum();
    let first = files
        .iter()
        .min_by(|a, b| {
            let s = |f: &FileRecording| {
                f.recording
                    .runs
                    .first()
                    .map_or(f64::INFINITY, |r| r.start.value())
            };
            s(a).total_cmp(&s(b))
        })
        .map_or(String::new(), |f| f.recording.first_stamp.clone());
    let mut s = String::new();
    let _ = write!(
        s,
        r#"<div class="stats">
<div class="stat"><div class="label">Files</div><div class="value">{}</div><div class="detail">{channels} pressure channel(s)</div></div>
<div class="stat"><div class="label">Recorded</div><div class="value">{:.1} s</div><div class="detail">from {}</div></div>
<div class="stat"><div class="label">Fronts the detector picked</div><div class="value">{}</div><div class="detail">default settings; nothing located (no network)</div></div>
</div>"#,
        files.len(),
        end - zero,
        esc(&first),
        chart_records(files)
            .iter()
            .map(|r| r.fronts.len())
            .sum::<usize>()
    );
    s.push_str(r#"<div class="card"><h2>Pressure</h2><p class="note">Every channel on one time axis, in seconds from the first sample. Breaks in a line are gaps in the data - never joined up. Lines mark the fronts the detector picked; shaded: not watching at full sensitivity. Hover to read every channel at once; drag across a chart to zoom.</p><p class="note">The detector first learns each sensor's noise from 2 s of data, so a stretch shorter than that is shaded as not watched. With no network given it cannot know how long echoes take to die away, so an echo may be marked as a front; the live engine, which knows the network, waits them out.</p>"#);
    let _ = write!(
        s,
        r#"<div class="controls" data-controls="rec-traces"><button type="button" class="chip" data-range="whole,0,{:.3}" aria-pressed="true">Whole recording</button><span class="hint">showing 0 - {:.2} s - drag across a chart to zoom</span></div><div class="js-traces" id="rec-traces" data-set="rec" data-from="0" data-to="{:.3}" data-zoomable></div></div>"#,
        end - zero,
        end - zero,
        end - zero
    );
    for f in files {
        let r = &f.recording;
        let _ = write!(
            s,
            r#"<div class="card"><h2>{}</h2><div class="facts"><div><div class="k">First sample</div><div class="v">{}</div></div><div><div class="k">Rate</div><div class="v">{:.2} S/s</div></div><div><div class="k">Samples</div><div class="v">{}</div></div><div><div class="k">Continuous runs</div><div class="v">{}</div></div></div><h3 style="margin-top:12px">What the reader decided ({})</h3><ul class="reasons">"#,
            esc(&f.name),
            esc(&r.first_stamp),
            1.0 / r.interval.value(),
            r.samples(),
            r.runs.len(),
            r.notes.len()
        );
        for n in &r.notes {
            match n.line {
                Some(l) => {
                    let _ = write!(s, "<li><b>line {l}</b>: {}</li>", esc(&n.text));
                }
                None => {
                    let _ = write!(s, "<li>{}</li>", esc(&n.text));
                }
            }
        }
        s.push_str("</ul></div>");
    }
    Page {
        id: "recordings",
        title: "Recordings".into(),
        crumb: format!("{} logger file(s), read by the logger reader", files.len()),
        source: "Logger files",
        body: s,
    }
}
