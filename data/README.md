# data/ - the quarantine for real data

Real data (network files, logger recordings, published datasets) comes from
outside this project. This folder keeps it contained and accounted for.

## The rules

1. **Nothing is downloaded without the owner's approval.** Each dataset is
   discussed first: what it is, who published it, its licence, its size,
   and what it would prove (validation rung 4, `docs/07_validation/`).
2. **Approved files land in `incoming/`**, exactly as downloaded. They are
   never opened by anything but our own readers, and never executed:
   no scripts, installers or macros from a dataset are ever run.
3. **Every file gets a line in `manifest.csv`** before it is used: where it
   came from, its licence, who approved it and when, its size, and its
   SHA-256 fingerprint. `.\data\verify.ps1` re-checks every fingerprint, so
   a file that changed - corrupted or tampered with - is noticed.
4. **Archives are unpacked into `datasets/<id>/`** only after approval and
   after listing what is inside them.
5. **Only this README, `manifest.csv` and `verify.ps1` are in git.** The data
   itself stays on this machine (see `.gitignore`); licences may not allow
   republishing it, and it can be large.
6. **Tests on real data live apart** (validation rung `datasets/`). When the
   data is not on the machine they are skipped and say so - they never
   fail silently or fetch anything.

In plain words:
- **Fingerprint (SHA-256).** A 64-character code worked out from every
  byte of a file. Change a single byte and the code comes out completely
  different. We write the code down the day a file arrives. Later,
  `verify.ps1` works it out again: the same code means the file is exactly
  what we approved, and a different code means it was damaged or altered.
- **Never run.** A dataset is only *read*, as numbers and text, by our
  own engine. If a download contains a program, script or macro (the lab
  dataset ships a small Python file), we do not run it.

## The datasets

Status on 2026-10-05: all three approved by the owner.
- **D1 and D2 are on this machine**, fingerprinted in `manifest.csv`. Every
  L-Town file also matches the checksum its publisher lists.
- **D3 is here too** (later the same evening, once Mendeley's server
  worked again): its fingerprint matches the one Mendeley publishes, the
  zip holds only CSV files, and only the 20 "Transient" runs (479 MB) are
  unpacked, into `datasets/tamu-testbed/`. The files read 25 600 times a
  second, not the 51.2 kHz the paper says.

### D1 - EPANET example networks (`epanet-examples`)

- **What:** the standard example network files (Net1, Net2, Net3) of
  EPANET, the US Environmental Protection Agency's pipe-network program.
  Net3 is based on a real water system. These are maps of pipes, pumps,
  tanks and demands, not recordings.
- **Source:** github.com/OpenWaterAnalytics/EPANET, folder
  `example-networks`. **Licence:** MIT (free, keep the notice).
- **Size:** a few kilobytes.
- **What it proves:** our EPANET reader (`jk network FILE.inp`) handles
  the files water engineers actually use; the steady-state solver
  matches EPANET's own published results for them.

### D2 - L-Town, the BattLeDIM 2020 leak benchmark (`ltown`)

- **What:** a town of 782 junctions and 905 pipes, laid out on a real
  town in Cyprus. It has two years (2018, 2019) of readings every
  5 minutes: pressures at 33 points (both counts checked in the files),
  flows, tank levels, and demands. It
  also has the truth: when each of its leaks started and how big it
  was.
- **Honest note:** the readings are *simulated* on that real layout (with
  realistic noise), not measured. At one reading every 5 minutes it is
  Tier 0 data: no pressure waves can be seen in it (principle P2).
- **Source:** Zenodo, record 4017659 (Vrachimis et al., BattLeDIM 2020).
  **Licence:** Creative Commons Attribution 4.0 (free, credit the
  authors).
- **Files to take (about 70 MB):** `L-TOWN.inp` (0.4 MB), and for each
  year `SCADA_Pressures.csv` (22.6 MB), `SCADA_Flows.csv` (4 MB),
  `SCADA_Levels.csv` (2.7 MB), `Leakages.csv` (6-10 MB), plus
  `README.txt` and `dataset_configuration.yaml`. Not taken: the 177 MB
  `L-TOWN_Real.inp` and the 92 MB `.xlsx` copies of the same readings.
- **What it proves:** the engine loads a town-sized network; the slow
  leak watch (night flows, CUSUM - `jk-balance`) finds leaks whose true
  start times are known, and we can count what it missed and when it
  raised a false alarm.

### D3 - Texas A&M leak testbed (`tamu-testbed`)

- **What:** a real laboratory pipe network: 47 m of 152.4 mm PVC pipe
  (schedule 80), laid out once as a loop and once as branches, fed by a
  pump. Two pressure sensors (PCB 102B16, P1 at the inlet, P2 at the far
  corner) record **51 200 times a second**, in pascals, for 30 s per
  run. In 40 of the runs (marked `T`) the outlet valve was shut fast by
  hand at about second 20, so the flow fell from 0.47 L/s to 0: **a
  real, measured pressure surge**, with real sensor noise.
- **Honest notes:**
  - These sensors measure pressure *changes*, not the pressure itself
    (they hold a step for minutes: discharge time constant >= 500 s,
    datasheet). So the low-pressure and boiling-point checks cannot be
    tested on this data.
  - The valve was shut by hand, likely slower than a wave takes to cross
    47 m of pipe, so the surge is expected to be small and gradual. Even
    an instant closure would make only about 1 m: velocity 0.47e-3 /
    (pi x 0.0762^2) = 0.026 m/s, times a wave speed of ~400 m/s
    [VERIFY for this pipe], over g = 9.81, gives 1.05 m.
  - Locating the source needs the pipe lengths, which are only in the
    paper's drawings.
- **Source:** Mendeley Data, doi 10.17632/tbrnp6vrnj.1; paper: Aghashahi,
  Sela and Banks, Data in Brief 48 (2023) 109148. **Licence:** Creative
  Commons Attribution 4.0.
- **File to take:** `Dynamic Pressure Sensor.zip` only (562 748 973
  bytes, about 563 MB). Not taken: the accelerometer (459 MB) and
  hydrophone (64 MB) zips, and the Python file.
- **What it proves:** on real recordings, the engine notices the surge,
  times its start, and calls a pressure *rise* a valve closing - not a
  burst. It also shows how the detector copes with real noise.

## Feeding the engine yourself

Open a terminal in the project folder (`C:\Users\HP\Desktop\joukowsky`)
and build once: `cargo build --release -p jk-cli`. Then:

| What | Command |
|---|---|
| Check every file is unchanged | `.\data\verify.ps1` |
| Read a network map, run its day or week | `.\target\release\jk network data\incoming\ltown\L-TOWN.inp` |
| The same for an EPANET example | `.\target\release\jk network data\incoming\epanet-examples\Net3.inp` |
| Read a recording file | `.\target\release\jk logger data\incoming\ltown\2018_SCADA_Levels.csv --unit m` |
| See it all in the viewer | `.\target\release\jk view --network data\incoming\ltown\L-TOWN.inp --recording data\incoming\ltown\2018_SCADA_Levels.csv --unit m`, then open `jk-view.html` |

`--unit m` is needed because L-Town's files do not name their units in
the column headings, and the engine never guesses units. In the viewer,
the menu gets two new groups: **Network file** (the map and every
assumption the reader made) and **Logger files** (the recording,
zoomable). L-Town's readings come every 5 minutes, so the viewer says
they are too slow to see a wave and looks for none (principle P2,
FN-046).

## What the real data has taught us so far

- **FN-045:** L-Town's model was refused, then read half right, in three
  ways where our reader was stricter or simpler than EPANET: a default
  pattern named but never defined, a pump curve ending at zero head, and
  homes with several types of demand. All fixed. After the fixes, the
  engine's pump switches within 0.2-3 h of the published data over a
  week, with no tuning.
- **FN-046:** the wave detector, given 5-minute data, marked 178
  "fronts" (the pump switching between readings). It now refuses
  anything slower than one reading a second.

## How a real-data test works

1. **Feed and look.** The recording is fed into the engine exactly as a
   site's logger files would be (`jk replay` with a site file pointing at
   the data). We read the report: what did it notice, when, what did it
   call it, how sure was it?
2. **Compare with the known truth.** For D3, the valve shut at about
   20 s, so the engine should say "valve closing" near 20 s. For D2, the
   leak list says when each leak began.
3. **Lock it in.** What we checked by eye becomes an automatic test in
   `tests/` (rung `datasets/`). Every later change to the engine is then
   checked against the real data too. If the data is not on the machine,
   those tests say "skipped: dataset not downloaded" - they never fail
   silently and never fetch anything.

Anything the real data shows to be wrong - in the engine or in the docs -
is fixed and logged in `docs/10_findings/`.

## Layout

```
data/
  README.md        these rules (tracked)
  manifest.csv     one line per approved file (tracked)
  verify.ps1       re-checks every fingerprint (tracked)
  incoming/        approved downloads, as received (not tracked)
  datasets/        unpacked, read-only use (not tracked)
```

## manifest.csv columns

`id, name, source, licence, approved_by, approved_on, file, bytes, sha256, notes`

- `file` is relative to `data/` (e.g. `incoming/ltown.zip`).
- `sha256` is `(Get-FileHash <file> -Algorithm SHA256).Hash`.
