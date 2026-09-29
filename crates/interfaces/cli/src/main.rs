//! `jk` - run joukowsky scenarios from the terminal.
//!
//! An interface crate (rule R5): it parses input, builds domain objects and
//! prints or writes results. No physics lives here.
//!
//! - `rpv`: the reservoir -> pipe -> valve transient.
//! - `view`: a web page to look at the engine working - a burst event and
//!   the placement advisor.
//! - `sim` (private): synthetic loggers and demo sites for `view`.

mod rpv;
mod sim;
mod view;

use std::process::ExitCode;

fn usage() -> String {
    format!(
        "jk - joukowsky engine\n\nUSAGE:{}{}",
        rpv::USAGE,
        view::USAGE
    )
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let result = match args.next().as_deref() {
        Some("rpv") => rpv::parse(args).and_then(|a| rpv::run(&a)),
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
