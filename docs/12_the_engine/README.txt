THE ENGINE - A GUIDE FOR REVIEWING IT
STATUS: DRAFT v0.1 (2026-10-01) - written for the owner's review of the
codebase before real-data testing; for anyone joining later too

The folders 00-11 are the PLAN and what building taught us. This folder
is the guide to what was actually BUILT: what the engine does, how one
alert is made, where everything lives in the code, how it was built, how
to run it, and what it cannot do yet. Plain words first; the details are
one step further in.

Reading order
-------------
01_the_engine_in_plain_words.txt   What it does and what problem it solves.
                                   Start here.
02_how_an_alert_is_made.txt        One burst, followed from the logger to
                                   the alert, step by step.
03_the_code_map.txt                Every folder: what it holds, which ones
                                   are the solvers, the domains, the rules
                                   between them, where to start reading.
04_how_it_was_built.txt            The story: method, order, milestones,
                                   what testing caught along the way.
05_how_to_run_and_review.txt       Commands, the viewer, replaying a site,
                                   and a suggested path through the code.
06_honest_limits.txt               What is proven, what is not, and what
                                   real data must still show.
07_running_the_tests.txt           Running the 363 tests step by step,
                                   what each test program proves, how to
                                   read the results, what to do if one
                                   fails.
08_real_data_how_to.txt            Feeding the engine real data: the
                                   commands, what the viewer shows, the
                                   engine against the published truth,
                                   checking the files are untouched.

Numbers on 2026-10-01
---------------------
  Code       about 33 000 lines of Rust in 13 crates, plus 13 500 lines of
             validation tests (117 files) - written from scratch, no
             outside libraries for the physics, signals or statistics.
  Tests      355 (348 quick, 7 slow), all passing; each prints what it
             measured next to what the physics predicts.
  Validation 35 numbered validation cases (V-A analytical, V-S
             simulation, V-X cross-code) in 07_validation.
  Findings   44 logged lessons (10_findings/findings_log.txt), 22 known
             gaps (10_findings/known_gaps.txt) - kept honest.
  History    82 commits, 2026-09-27 to 2026-10-01.
