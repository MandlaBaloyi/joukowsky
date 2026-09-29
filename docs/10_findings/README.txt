FINDINGS: WHAT BUILDING AND TESTING TAUGHT US
STATUS: DRAFT v0.1 - living folder, added to as we work

Why this folder exists
----------------------
The other folders (00-09) are the plan: what we believed before building.
They are hints, not limits. Building teaches things no plan foresaw - a
bug that only real timestamps expose, a junction that matters more than
a kilometre of pipe, a factor nobody wrote down. This folder is where
those discoveries are kept, so they are never lost and never repeated.

Standing rule (owner, 2026-09-29): anything missing or wrong in the docs
is fixed in the docs as soon as it is found - in the topic file where it
belongs - and logged here with the date. "It is not in the docs" is never
a reason not to do something.

Files
-----
findings_log.txt           Every discovery, dated, numbered FN-nnn: what
                           happened, what it taught, what changed.
known_gaps.txt             What is NOT built or NOT proven yet, honestly,
                           numbered G-nn, with how each will be closed.
                           Read this before claiming anything to a client.
detection_range_and_scale.txt
                           How far a burst can be felt, what limits the
                           kilometres we cover, and how to extend them.
external_factors.txt       Weather, lightning, heat, tremors, vibration,
                           power: what each does to the data and to the
                           pipes, and what the engine does about it.
real_data_candidates.txt   Real datasets we could use, what each would
                           prove, and what must be checked before the
                           owner approves a download. Nothing is fetched
                           without approval.

How to add a finding
--------------------
1. Append to findings_log.txt: next FN number, date, one-line title, then
   WHAT HAPPENED / WHAT IT TAUGHT / WHAT CHANGED (code, test, doc).
2. Fix the topic doc it touches (e.g. 04_/burst_localisation.txt) with an
   "Implementation note" and point back to the FN number.
3. If it opens a gap, add it to known_gaps.txt; if it closes one, mark
   the gap CLOSED with the date and the test that proves it.
4. Numbers follow P8: cite them, show the working, or mark [VERIFY].
