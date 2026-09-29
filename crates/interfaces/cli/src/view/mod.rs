//! `jk view`: a web page to look at the engine working.
//!
//! One self-contained HTML file - open it in any browser; nothing is
//! installed, served or downloaded. Two sections, both on simulated sites
//! until the data readers exist:
//! 1. [`event`]: a burst as the live engine saw it - alerts, maps, traces.
//! 2. [`placement`]: where loggers should go on a campus, and what each buys.
//!
//! The pieces: [`page`] (theme, type, tooltips), [`charts`] (the pictures
//! as inline SVG), [`svg`] (escaping, scales, ticks).

mod charts;
mod event;
mod page;
mod placement;
mod svg;

use std::path::PathBuf;

pub const USAGE: &str = "
    jk view [OPTIONS]     write a web page showing the engine at work

  OPTIONS:
    --out FILE            where to write the page [jk-view.html]
    --loggers N           loggers for the placement advisor, 2-8 [6]
    --no-placement        skip the placement section (it takes about a minute)
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
    eprintln!("jk view: simulating the estate burst and running the live engine...");
    let mut body = String::from(
        r#"<div class="wrap"><h1>joukowsky - engine viewer</h1>
<p>The engine at work on simulated sites: what it detects, where it says a burst is, and where
loggers should go. Everything here is computed by the same code the tests validate.</p>"#,
    );
    body.push_str(&event::section()?);
    if a.placement {
        body.push_str(&placement::section(a.loggers)?);
    }
    body.push_str("</div>");
    let html = page::page("joukowsky viewer", &body);
    std::fs::write(&a.out, html).map_err(|e| format!("cannot write {}: {e}", a.out.display()))?;
    println!("Wrote {} - open it in a browser.", a.out.display());
    Ok(())
}
