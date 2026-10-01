//! `jk` - run joukowsky scenarios from the terminal.
//!
//! An interface crate (rule R5): it parses input, builds domain objects and
//! prints or writes results. No physics lives here.
//!
//! - `rpv`: the reservoir -> pipe -> valve transient.
//! - `network`: read an EPANET `.inp` network file and summarise it.
//! - `logger`: read a logger export (CSV) and summarise it.
//! - `replay`: run the live engine over a site's recordings (a site file).
//! - `scale`: time every part of the engine on synthetic towns (V-S4).
//! - `view`: a web page to look at the engine working - a burst event and
//!   the placement advisor.
//! - `sim` (private): synthetic loggers and demo sites for `view`.

mod logger;
mod network;
mod replay;
mod rpv;
mod scale;
mod sim;
mod view;

use std::process::ExitCode;

fn usage() -> String {
    format!(
        "jk - joukowsky engine\n\nUSAGE:{}{}{}{}{}{}",
        rpv::USAGE,
        network::USAGE,
        logger::USAGE,
        replay::USAGE,
        scale::USAGE,
        view::USAGE
    )
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let result = match args.next().as_deref() {
        Some("rpv") => rpv::parse(args).and_then(|a| rpv::run(&a)),
        Some("logger") => logger::parse(args).and_then(|a| logger::run(&a)),
        Some("replay") => replay::parse(args).and_then(|a| a.map_or(Ok(()), |a| replay::run(&a))),
        Some("scale") => scale::parse(args).and_then(|a| scale::run(&a)),
        Some("network") => network::parse(args).and_then(|f| network::run(&f)),
        Some("view") => view::parse(args).and_then(|a| view::run(&a)),
        Some("-h" | "--help") | None => {
            print!("{}", usage());
            return ExitCode::SUCCESS;
        }
        Some(other) => Err(format!("unknown command '{other}'\n\n{}", usage())),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("jk: {e}");
            ExitCode::from(2)
        }
    }
}
