//! `jk view`: a web page to look at the engine working.
//!
//! One self-contained HTML file with a sidebar menu - open it in any
//! browser; nothing is installed, served or downloaded (TD-019). Until the
//! data readers exist, it shows the engine on simulated sites.
//!
//! - [`model`]: the engine run once on the demo sites.
//! - [`pages`]: one page per menu entry.
//! - [`shell`]: the frame - sidebar, title bar, theme; `assets/` holds its
//!   stylesheet and script, built into the binary.
//! - [`charts`]: pictures drawn here as SVG (timeline, maps, columns);
//!   pressure traces are drawn by the page script so they can be zoomed.
//! - [`svg`]: escaping, scales, ticks, JSON.

mod charts;
mod model;
mod pages;
mod shell;
mod svg;

use std::path::PathBuf;

use model::{EventModel, PlacementModel};
use shell::{NavGroup, NavItem, Page};

pub const USAGE: &str = "
    jk view [OPTIONS]     write a web page showing the engine at work

  OPTIONS:
    --out FILE            where to write the page [jk-view.html]
    --loggers N           loggers for the placement advisor, 2-8 [6]
    --no-placement        skip the placement pages
";

/// What `jk view` was asked for.
pub struct Args {
    out: PathBuf,
    loggers: usize,
    placement: bool,
}

/// Reads the options after `view`.
///
/// # Errors
/// An unknown option or a bad value.
pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args {
        out: PathBuf::from("jk-view.html"),
        loggers: 6,
        placement: true,
    };
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--no-placement" => a.placement = false,
            "--out" => a.out = PathBuf::from(it.next().ok_or("--out needs a file name")?),
            "--loggers" => {
                let v = it.next().ok_or("--loggers needs a number")?;
                a.loggers = v
                    .parse()
                    .ok()
                    .filter(|n| (2..=8).contains(n))
                    .ok_or_else(|| format!("--loggers: '{v}' is not a whole number from 2 to 8"))?;
            }
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(a)
}

/// Builds the page and writes it.
///
/// # Errors
/// A failed scenario (a bug) or a file that cannot be written.
pub fn run(a: &Args) -> Result<(), String> {
    eprintln!("jk view: simulating the estate bursts and running the live engine...");
    let event = EventModel::run()?;
    let placement = if a.placement {
        eprintln!("jk view: choosing {} loggers for the campus...", a.loggers);
        Some(PlacementModel::run(a.loggers)?)
    } else {
        None
    };

    let mut nav = vec![
        NavGroup {
            heading: "Start",
            site: None,
            items: vec![NavItem {
                id: "overview",
                label: "Overview",
                count: None,
            }],
        },
        NavGroup {
            heading: "Event",
            site: Some("estate".into()),
            items: vec![
                NavItem {
                    id: "alerts",
                    label: "Alerts",
                    count: Some(event.issued.len()),
                },
                NavItem {
                    id: "loggers",
                    label: "Logger records",
                    count: Some(event.records.len()),
                },
                NavItem {
                    id: "network",
                    label: "Network",
                    count: None,
                },
            ],
        },
    ];
    let mut pages: Vec<Page> = vec![
        pages::overview::page(&event, placement.as_ref()),
        pages::alerts::page(&event),
        pages::loggers::page(&event),
        pages::network::page(&event),
    ];
    if let Some(p) = &placement {
        nav.push(NavGroup {
            heading: "Placement",
            site: Some("campus".into()),
            items: vec![
                NavItem {
                    id: "placement",
                    label: "Where the loggers go",
                    count: None,
                },
                NavItem {
                    id: "placement-value",
                    label: "What each logger buys",
                    count: None,
                },
                NavItem {
                    id: "placement-pipes",
                    label: "Pipe by pipe",
                    count: None,
                },
            ],
        });
        pages.push(pages::placement::recommendation(p));
        pages.push(pages::placement::value(p));
        pages.push(pages::placement::pipes(p));
    }
    nav.push(NavGroup {
        heading: "Help",
        site: None,
        items: vec![NavItem {
            id: "about",
            label: "About the data",
            count: None,
        }],
    });
    pages.push(pages::about::page());

    let html = shell::document(&nav, &pages, &pages::data_json(&event));
    std::fs::write(&a.out, html).map_err(|e| format!("cannot write {}: {e}", a.out.display()))?;
    println!("Wrote {} - open it in a browser.", a.out.display());
    Ok(())
}
