# joukowsky

**Software that listens to water pipes, and tells you within seconds that one has burst, where, and how big.**

South Africa loses close to half of its treated water before it is billed (about 47% in the national No Drop reports), much of it through leaks and bursts that are found hours or days late, usually when someone reports them. `joukowsky` is an engine that takes a site's pipe network and its pressure loggers, listens at 128 samples per second, and works out what happened from the physics of the pressure wave that every burst sends through the water.

> Status: a research prototype, built from 27 Sept 2026. Validated against exact physics, simulated sites and public benchmark datasets. **Not yet tested on a live network.** See [Honest limits](#honest-limits).

## What it does

When a pipe bursts, a pressure wave races along the water at roughly 1 km/s and reaches each logger at a slightly different moment. The engine uses those arrival times:

| Step | What the engine answers | Where |
|---|---|---|
| **Detect** | Did something just happen? (robust noise model, CUSUM change detection, step-area onset) | `crates/domain/detection` |
| **Classify** | A burst, a valve closing, a pump trip, a pipe refilling after a supply cut? | `crates/domain/classification` |
| **Locate** | Where on the pipe map, as a probability distribution, not a single guess | `crates/domain/localisation` |
| **Size** | How large is the burst, from the measured pressure drops | `crates/domain/localisation` |
| **Slow leaks** | Night-flow balance and pressure-sensor fitting for leaks with no wave at all | `crates/domain/balance`, `localisation/pressure.rs` |
| **Wear** | Rainflow counting and Miner damage: which pipes are being fatigued by pressure shocks | `crates/domain/health` |
| **Plan** | Where to place loggers before buying them, and which spots stay blind | `localisation/placement` |

Every answer carries a confidence, and the engine is allowed to say *"I do not know"* instead of guessing.

```
loggers / SCADA --> telemetry (gaps kept, never silently filled)
                        |
                        v
                   detection --> classification --> localisation + sizing
                        |                                   |
                        v                                   v
                  fatigue + supply watch              alert (with revisions)
                                                            |
                                                            v
                                                   AlertSink (a port: a collector today,
                                                   a webhook / MQTT / OPC UA tomorrow)
```

## Why "Joukowsky"

Nikolai Zhukovsky ("Joukowsky" in French) is remembered for aerodynamics, but in 1897-98 he also solved water hammer: after unexplained pipe failures in Moscow he ran full-scale experiments on the city's mains and derived the relation between a change in flow speed and the pressure jump it causes, `dp = rho * a * dv`. That equation is the starting point of this engine. More in [`docs/00_why_joukowsky`](docs/00_why_joukowsky/the_name.txt).

## How it is built

- **Physics first.** Every model has a physical meaning; a result that breaks conservation of mass or momentum is treated as a bug.
- **Built from scratch.** The workspace has **zero external crates** (see `Cargo.lock`). The solvers, linear algebra (dense and sparse Cholesky), random numbers, complex numbers, signal processing and statistics are all written in the project.
- **Clean architecture.** Bounded contexts (hydraulics, detection, localisation, classification, balance, channels, health) stay pure: no I/O, no clocks. Files, EPANET networks and logger exports come in through adapters; alerts leave through a port.
- **The solvers.** Method of characteristics for the wave (single pipes and whole networks, with vapour and gas cavities, viscoelastic pipe walls, pumps, valves, check valves, tanks, leaks), an independent transfer-matrix model as a cross-check, a steady-state network solver (1 000 km in 0.3 s), and a 1-D Saint-Venant solver for canals and rivers.
- **Expect broken data.** Gaps, late uploads, bad clocks, load-shedding power cuts and drifting clocks are handled explicitly.
- **Every claim has a test.** Tests print what they measured next to what the physics predicts, so the logs can be read as a lesson. Each function names the document it implements.

About **50 000 lines of Rust** in 14 workspace members; **372 tests** in the full suite (361 in the quick run), all passing at the last recorded run (6 Oct 2026, [`tests/logs`](tests/logs)); 30+ numbered validation cases; 54 logged findings and 24 tracked known gaps.

## Results so far

- **Exact physics:** wave periods, Joukowsky jumps, friction, plastic walls, pumps, valves, tanks, canals, refilling, and vapour and gas cavities are checked against analytical answers, most to better than one part in a million.
- **Independent cross-check:** the wave solver agrees with a transfer-matrix model within 0.15% on a branched network.
- **Simulated sites:** bursts of 3-12 mm on a simulated estate were detected 100% of the time, called bursts 95-100%, located within about 0.1-0.4 m when the wave speed is known (5-7 m with a 3% wave-speed error), sized within 1-7%, with no false alerts in 9 logger-hours of quiet.
- **Public benchmarks:** the engine has been run on EPANET's example networks and on **L-Town** (the BattLeDIM 2020 leak-detection competition: a 2.7 km town, 33 pressure sensors, two years of data, with the 33 true leaks published so results can be scored). These were the first files made by someone else, and they exposed bugs the project's own simulations never showed; each is written up and fixed ([`FN-045 to FN-054`](docs/10_findings/findings_log.txt)). From its own night-flow alarms the engine located a changed leak within 300 m in 17 of 19 alarms (median 153 m in a 2.7 km town, against 836 m for a random guess). The log also records the negative results, such as growing leaks, where the signal and not the timing is the problem.

## Honest limits

Most of the evidence above is from simulations (this project's own, and the competition organisers' for L-Town), cross-checked by an independent method but not yet by laboratory measurements or a live network. The false-alert rate on a real network is unknown, real wave speeds must be measured, pump starts and valve openings are not yet classified (a valve opening looks like a burst), and location on a large municipal network is untested. These are written down, with the tests that would close them, in [`docs/12_the_engine/06_honest_limits.txt`](docs/12_the_engine/06_honest_limits.txt) and [`docs/10_findings/known_gaps.txt`](docs/10_findings/known_gaps.txt).

## Run it

You need Rust 1.98.1 (pinned in `rust-toolchain.toml`). No other dependencies and no internet.

```
cargo test --workspace                    # all tests; real-data tests skip when the data is absent
cargo run -p jk-cli -- rpv                # a reservoir-pipe-valve water hammer demo
cargo run --release -p jk-cli -- view     # writes jk-view.html: overview, alerts, loggers, network,
                                          # logger placement, supply, leaks, pipe health
cargo run --release -p jk-cli -- scale --km 10 --advise
```

On Windows, `.\tests\run-tests.ps1 -Suite quick` runs the test programs one by one and keeps a readable log. The real-data datasets are not included (their licences and size); how they are fetched, fingerprinted and kept apart is in [`data/README.md`](data/README.md), and the `view --real` pages need them.

## Repository map

```
crates/kernel/                 typed SI units, random numbers, linear algebra, clocks
crates/domain/hydraulics/      Joukowsky relations, MOC solvers, network + steady-state solvers
crates/domain/telemetry/       logger chunks into continuous stretches with explicit gaps
crates/domain/detection/       noise model, baseline, CUSUM, onset, live detector stream
crates/domain/classification/  rules over physical features
crates/domain/localisation/    travel times, posterior over the network, sizing, placement
crates/domain/balance/         night flows and slow-leak watch
crates/domain/channels/        canals and rivers (Saint-Venant)
crates/domain/health/          rainflow counting, fatigue damage
crates/application/            the live monitor, alerts, replay of a site's recordings
crates/adapters/files/         EPANET networks, logger CSVs, SCADA tables, site files
crates/sim/                    synthetic towns for testing
crates/interfaces/cli/         the `jk` command and the viewer
tests/                         validation tests, by context and rung (analytical, invariants, simulation, datasets)
docs/                          physics, design, validation strategy, findings, a guide to what was built
```

New here? Read [`docs/12_the_engine/01_the_engine_in_plain_words.txt`](docs/12_the_engine/01_the_engine_in_plain_words.txt), then [`02_how_an_alert_is_made.txt`](docs/12_the_engine/02_how_an_alert_is_made.txt).

## About

Built by **Mandla Baloyi** in South Africa, with [Claude Code](https://claude.com/claude-code) as a coding assistant. I set the problem, the principles and the validation rules and review every result; the code is written to be read, and I am learning Rust as I go. Contact me through my GitHub profile.

## Licence

All rights reserved. The code is published for reading and review; see [`LICENSE`](LICENSE).
