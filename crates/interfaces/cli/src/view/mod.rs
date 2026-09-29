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
    --network FILE        also show a network read from an EPANET .inp file
";

/// What `jk view` was asked for.
pub struct Args {
    out: PathBuf,
    loggers: usize,
    placement: bool,
    network: Option<PathBuf>,
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
        network: None,
    };
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--no-placement" => a.placement = false,
            "--out" => a.out = PathBuf::from(it.next().ok_or("--out needs a file name")?),
            "--network" => {
                a.network = Some(PathBuf::from(
                    it.next().ok_or("--network needs a file name")?,
                ));
            }
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

fn item(id: &'static str, label: &'static str, count: Option<usize>) -> NavItem {
    NavItem { id, label, count }
}

/// Builds the page and writes it.
///
/// # Errors
/// A failed scenario (a bug), a network file that cannot be read, or a
/// page that cannot be written.
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
            items: vec![item("overview", "Overview", None)],
        },
        NavGroup {
            heading: "Event",
            site: Some("estate".into()),
            items: vec![
                item("alerts", "Alerts", Some(event.issued.len())),
                item("loggers", "Logger records", Some(event.records.len())),
                item("network", "Network", None),
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
                item("placement", "Where the loggers go", None),
                item("placement-value", "What each logger buys", None),
                item("placement-pipes", "Pipe by pipe", None),
            ],
        });
        pages.push(pages::placement::recommendation(p));
        pages.push(pages::placement::value(p));
        pages.push(pages::placement::pipes(p));
    }
    if let Some(path) = &a.network {
        let (group, page) = network_file(path)?;
        nav.push(group);
        pages.push(page);
    }
    nav.push(NavGroup {
        heading: "Help",
        site: None,
        items: vec![item("about", "About the data", None)],
    });
    pages.push(pages::about::page());

    let html = shell::document(&nav, &pages, &pages::data_json(&event));
    std::fs::write(&a.out, html).map_err(|e| format!("cannot write {}: {e}", a.out.display()))?;
    println!("Wrote {} - open it in a browser.", a.out.display());
    Ok(())
}

/// The menu group and page for a network read from an `.inp` file.
fn network_file(path: &std::path::Path) -> Result<(NavGroup, Page), String> {
    let inp = jk_files::epanet::read(path, &jk_files::epanet::InpOptions::default())
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let name = path.file_name().map_or_else(
        || path.display().to_string(),
        |n| n.to_string_lossy().into_owned(),
    );
    let group = NavGroup {
        heading: "Network file",
        site: Some(name.clone()),
        items: vec![item("file", "Map and notes", Some(inp.notes.len()))],
    };
    Ok((group, pages::file::page(&inp, &name)))
}
