# joukowsky tests

The engine is only as trustworthy as the tests that prove it. This folder
holds the **validation tests**: each one checks the engine against physics
we already know the answer to, and prints the comparison so you can read it.

## Layout

Tests are grouped in three layers, so the folder stays navigable as it grows
to thousands of tests (and hundreds of thousands of generated cases):

1. **Bounded context** - mirrors the crates: `kernel`, `hydraulics`, later
   `detection`, `localisation`, `twin`, ... When something breaks, you know
   which part of the engine to look at.
2. **Validation rung** - how strong the evidence is
   (`docs/07_validation/validation_strategy.txt`):
   - `analytical/` - answers physics gives in closed form (Joukowsky, 2L/a)
   - `invariants/` - laws that must always hold (steady state stays steady,
     the grid does not change the answer, mass is conserved); these grow into
     property tests over thousands of random networks
   - `cross_code/` - our solver against TSNet (later)
   - `datasets/`, `golden/` - real recorded events, bit-for-bit replays (later)
3. **Topic** - `single_pipe/`, `joukowsky/`, later `boundaries/`, `network/`, ...

```
tests/
├── Cargo.toml              makes this folder a crate (jk-tests); one [[test]] per context
├── run-tests.ps1           runs the tests and saves a log
├── common/lib.rs           shared helpers: check_close, check, standard_system
├── kernel/                 physical quantities
│   └── conversions.rs
├── hydraulics/
│   ├── main.rs             the context's table of contents
│   ├── analytical/
│   │   ├── joukowsky/      surge by material, burst sizing
│   │   └── single_pipe/    va1 square wave, va2 slow closure, friction
│   └── invariants/         steady state, grid independence
└── logs/                   one log per run; latest.log is the most recent
```

**Why one test program per context, not per folder:** Rust compiles and
links every test program separately, and linking is the slow part. Topics
are therefore folders (modules) *inside* a context's program. That keeps
builds fast on a laptop no matter how many tests we add.

**Traceability:** a test whose claim is in the validation strategy starts
with its ID - `va1_...` is V-A1, `va2_...` is V-A2 - so a log line leads
straight back to the doc it proves.

Small unit tests (does a function reject bad input, does a formula hold)
stay inside the code files, in blocks marked `#[cfg(test)]` - the Rust
convention. They run together with these.

## Two suites

| Suite | What runs | When |
|---|---|---|
| `quick` (default) | everything except tests marked `#[ignore]` | every change; takes seconds |
| `full` | also the slow and exhaustive tests | before committing physics changes; later nightly in the cloud |

A test joins the full suite by adding `#[ignore = "full suite: why it is slow"]`
under its `#[test]` line.

## Running

From the repository root, in the VS Code terminal:

| What | Command |
|---|---|
| Quick suite, saved log | `.\tests\run-tests.ps1` |
| Full suite, saved log | `.\tests\run-tests.ps1 -Suite full` |
| One context | `.\tests\run-tests.ps1 -Only hydraulics` |
| One topic or test | `.\tests\run-tests.ps1 -Filter single_pipe` or `-Filter va1` |
| Quick, no log | `cargo test --workspace` |

Then open `tests/logs/latest.log`.

## Reading a log

```
test analytical::single_pipe::square_wave::va1_frictionless_instant_closure_is_a_joukowsky_square_wave ... Grid and timing:
  time step dt [s]                             got       0.050000   expected       0.050000   rel.err 0.0e0
  ...
ok
```

- `test <rung>::<topic>::<file>::<test name>` - which test ran, and where it lives.
- Each indented line is one check: what was measured (`got`), what the
  physics predicts (`expected`), and the relative error between them.
- `ok` means every check passed. A failure shows `FAILED` and a `panicked at`
  line naming the file, line number and the check that broke.
- The footer counts passed, failed, skipped (slow tests outside the suite)
  and filtered tests.

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

Folders are modules: each folder has a `mod.rs` (or `main.rs` at the top)
listing its files with `mod <name>;`. A file not listed there is not compiled.

## Adding tests

- **New test in an existing topic:** add a `#[test]` function to the file.
- **New file:** create `<name>.rs` in the topic folder, start it with a `//!`
  comment saying what physics it checks and which doc it comes from, and add
  `mod <name>;` to that folder's `mod.rs`.
- **New topic:** create the folder with a `mod.rs`, and add `mod <folder>;`
  to the rung's `mod.rs`.
- **New bounded context:** create `tests/<context>/main.rs`, add a `[[test]]`
  entry to `tests/Cargo.toml`, and add the name to `-Only` in `run-tests.ps1`.
