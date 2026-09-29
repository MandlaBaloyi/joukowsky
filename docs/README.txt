JOUKOWSKY - RESEARCH & DESIGN DOCUMENTATION
STATUS: DRAFT v0.1 - for review

The problem we are solving
--------------------------
Water pipes burst and nobody knows until days later. Water is lost, and the
pipes keep getting damaged by sudden pressure shocks ("water hammer") that
nobody can see, because the pipes are not being watched closely enough.

joukowsky watches the pressure in the pipes, very fast and all the time.
When a pipe bursts, or a pressure shock hits, it notices within seconds,
tells you where it happened, and warns you before more damage is done.
Over time it learns how each network behaves, so it keeps getting better.

Reading order
-------------
00_why_joukowsky/            Why the project carries this name; mission and
                             engineering principles.
01_problem_definition/       The problem, stated plainly and then formally.
02_physics_of_transients/    The physics the engine is built on: Joukowsky's
                             equation, wave speed, the governing PDEs, MOC,
                             cavitation, friction, pressure-leakage laws.
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
11_business/                 How the engine meets the world: loggers and
                             hardware partners, the pilot kit, markets
                             (sites, mines, municipalities). Owner's
                             decisions.
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
