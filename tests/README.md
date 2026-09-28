# joukowsky tests

The engine is only as trustworthy as the tests that prove it. This folder
holds the **validation tests**: each one checks the engine against physics
we already know the answer to, and prints the comparison so you can read it.

## Layout

```
tests/
├── Cargo.toml          makes this folder a Rust crate (jk-tests) in the workspace
├── run-tests.ps1       runs the tests and saves a log
├── common/lib.rs       shared helpers: check_close, check, standard_system
├── 01_units/           the physical quantities (head, pressure, flow, wave speed)
├── 02_joukowsky/       the surge formulas, against every worked number in the docs
├── 03_single_pipe/     reservoir -> pipe -> valve, solved by MOC (V-A1, V-A2)
└── logs/               a dated log per run; latest.log is the most recent
```

Folders are numbered in the order you should read them: each builds on the
one before. New folders get the next number (04_boundaries, 05_network, ...).

Small unit tests (does a function reject bad input, does a formula hold)
stay inside the code files, in blocks marked `#[cfg(test)]` - that is the
Rust convention. They run together with these.

## Running

From the repository root, in the VS Code terminal:

| What | Command |
|---|---|
| Everything, with a saved log | `.\tests\run-tests.ps1` |
| One folder, with a saved log | `.\tests\run-tests.ps1 -Only single_pipe` |
| Everything, quick, no log | `cargo test --workspace` |
| One folder, showing its printout | `cargo test -p jk-tests --test joukowsky -- --nocapture` |
| One test by name | `cargo test square_wave -- --nocapture` |

Then open `tests/logs/latest.log`.

## Reading a log

A passing test looks like this:

```
test square_wave::frictionless_instant_closure_is_a_joukowsky_square_wave ... Grid and timing:
  time step dt [s]                             got       0.050000   expected       0.050000   rel.err 0.0e0
  ...
ok
```

- `test <file>::<test name>` - which test ran.
- Each indented line is one check: what was measured (`got`), what the
  physics predicts (`expected`), and the relative error between them.
- `ok` means every check passed. A failure shows `FAILED` and a `panicked at`
  line naming the file, line number and the check that broke.
- The footer counts passed and failed tests across the whole run.

## How a Rust test works

```rust
#[test]                                   // marks a function as a test
fn one_bar_is_about_10_2_metres_of_water() {
    let head = Head::from_pressure(Pressure::from_bar(1.0), Density::new(998.0), STANDARD_GRAVITY);
    check_close("head of 1 bar [m]", head.value(), 10.2, 0.01);   // fails the test if >1% off
}
```

`cargo test` finds every `#[test]` function, runs it, and reports `ok` or
`FAILED`. A test fails if it panics - which is what `assert!`, `check` and
`check_close` do when a claim is false. `println!` output is hidden unless
you pass `--nocapture` (the script always does).

## Adding a test

1. Pick the folder for the topic, or create the next numbered one.
2. Add a `.rs` file with `#[test]` functions; start it with a `//!` comment
   saying what physics it checks and which doc it comes from.
3. List the file as `mod <name>;` in that folder's `main.rs`.
4. A new folder also needs a `[[test]]` entry in `tests/Cargo.toml` and its
   name added to the `-Only` list in `run-tests.ps1`.
