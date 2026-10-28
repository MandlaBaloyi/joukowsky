JOUKOWSKY - RESEARCH & DESIGN DOCUMENTATION
STATUS: DRAFT v0.1 - for review

The problem we are solving (the owner's statement, 2026-10-02)
--------------------------
South Africa loses close to half its treated water before it is billed,
much of it through leaks and burst pipes [VERIFY: about 47% non-revenue
water, No Drop report - 01_problem_definition/the_scale_of_nrw.txt].
Today a burst is usually found hours or days later, often only when
someone reports it, and by then a lot of water is gone and the damage is
done. Joukowsky is software that changes that. It takes a site's pipe
network, advises where to place pressure loggers, and listens to them 128
times a second. When a pipe bursts or a pressure shock hits, the wave
reaches the loggers in under a second. Within seconds the engine says
what happened, where it is to within metres, how big it is, and how sure
it is, straight into the site's own control room. That saves water,
repair costs and downtime, and it shows which pipes are being worn out by
the shocks before they fail.

Over time it will learn how each network behaves, so it keeps getting
better.

Reading order
-------------
00_why_joukowsky/            Why the project carries this name; mission and
                             engineering principles.
01_problem_definition/       The problem, stated plainly and then formally.
02_physics_of_transients/    The physics the engine is built on: Joukowsky's
                             equation, wave speed, the governing PDEs, MOC,
                             cavitation, friction, pressure-leakage laws;
                             and the sources upstream - dams, rivers,
                             canals, bulk supply (sources_dams_rivers_
                             canals.txt).
03_sensing_and_signal/       Why current SCADA cannot see transients, and what
                             we need from the data sources we connect to (we
                             build software, not sensors).
04_detection_and_localisation/
                             Detecting events, classifying them, and locating
                             them on the network graph.
05_learning_systems/         The digital twin and the models that learn over
                             time.
06_engine_architecture/      How the pieces fit: data -> core -> actions.
07_validation/               How we prove each claim, from textbook cases to
                             live partner data.
08_research/                 References, open questions, and the research
                             backlog.
09_technologies/             What we build with (and why), what we must
                             invent ourselves, and the decision log.
10_findings/                 What building and testing taught us: the
                             findings log, known gaps (read before any
                             claim), detection range and scale, external
                             factors (weather, lightning, tremors), real
                             data candidates. Living folder.
12_the_engine/               What was actually BUILT: the engine in plain
                             words, how one alert is made, the code map
                             (where the solvers and domains live), how it
                             was built, how to run and review it, and its
                             honest limits. Start here to review the code.
glossary.txt                 The words we use, in plain language.

These docs are hints, not limits
--------------------------------
00-09 are the plan as first written. Building discovers what the plan
missed. Anything missing or wrong is fixed in the topic file where it
belongs, as soon as it is found, and logged in 10_findings/
findings_log.txt with the date (owner's standing rule, 2026-09-29).

Conventions
-----------
- Plain text. Formulas in ASCII. SI units throughout.
  H = piezometric head [m], p = pressure [Pa], Q = flow [m^3/s],
  V = mean velocity [m/s], a = wave speed [m/s], g = 9.81 m/s^2,
  rho = water density (~998 kg/m^3 at 20 C).
  1 bar = 100 kPa ~= 10.2 m of water head.
- Every file carries a STATUS line. DRAFT = open for review;
  REVIEWED = accepted by the owner; FROZEN = changes need a decision record.
- Numbers taken from outside sources are cited in 08_research/references.txt.
  Anything marked [VERIFY] has not yet been checked against a primary source.
- Numbers we derived ourselves show the working so they can be checked.
