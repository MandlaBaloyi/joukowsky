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
