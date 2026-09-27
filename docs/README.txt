JOUKOWSKY - RESEARCH & DESIGN DOCUMENTATION
STATUS: DRAFT v0.1 - for review

What this is
------------
joukowsky is a real-time transient-detection engine for water distribution
networks. It listens to pressure at high frequency, recognises the physical
signature of bursts (sudden pressure drops) and water hammer (sudden pressure
spikes), locates them on the pipe network, and learns the network's behaviour
over time.

This is not a CRUD application. The core is physics (1-D unsteady pipe flow),
signal processing (change detection at millisecond resolution), inverse
problems (where did the wave come from?), and learning systems (models that
improve with every event the network produces). Business models come later;
this documentation is about solving the problem first.

Reading order
-------------
00_why_joukowsky/            Why the project carries this name; mission and
                             engineering principles.
01_problem_definition/       The problem, stated plainly and then formally.
02_physics_of_transients/    The physics the engine is built on: Joukowsky's
                             equation, wave speed, the governing PDEs, MOC,
                             cavitation, friction, pressure-leakage laws.
03_sensing_and_signal/       Why current SCADA cannot see transients, and the
                             sensing, sampling and clock requirements that can.
04_detection_and_localisation/
                             Detecting events, classifying them, and locating
                             them on the network graph.
05_learning_systems/         The digital twin and the models that learn over
                             time.
06_engine_architecture/      How the pieces fit: edge -> core -> actions.
07_validation/               How we prove each claim, from textbook cases to
                             field pilots.
08_research/                 References, open questions, and the research
                             backlog.

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
