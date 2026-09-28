09_TECHNOLOGIES - WHAT WE BUILD WITH, AND WHAT WE MUST BUILD OURSELVES
STATUS: DRAFT v0.1 - for review (living folder; more files will be added)

Purpose
-------
Every technology in the engine is chosen for a reason written down here:
what it does for us, what we rejected, and what would make us change our
mind. Production grade means no library enters the engine by accident.

This folder also records where the tools STOP - problems for which no
library, paper or product gives an answer, and where we will have to
create our own solvers and models (11_frontier_what_we_must_invent.txt).

Ring legend (every technology carries one)
------------------------------------------
  ADOPT   Decided. Use it now. Changing it needs a decision record (12_).
  TRIAL   Use it in prototypes; promote to ADOPT or drop after evidence.
  ASSESS  Worth studying; not yet in any code.
  HOLD    Do not use now (licence, maturity, cost or fit). Reason given.
  BUILD   Nothing adequate exists; we write it ourselves.

Files
-----
  01_language_and_toolchain.txt      Rust first; Python later; toolchain.
  02_numerics_and_solvers.txt        Arrays, linear algebra, FFT,
                                     parallelism, floating-point
                                     determinism, the solvers we write.
  03_networks_and_reference_tools.txt
                                     Pipe-network graphs, EPANET files,
                                     reference simulators, open datasets.
  04_signal_processing_and_detection.txt
                                     Detectors, onset pickers, modal fits.
  05_statistics_and_inference.txt    Bayesian inference, MCMC, filtering,
                                     calibration of uncertainty, fatigue
                                     counting.
  06_machine_learning_and_ai.txt     Training stack, inference in Rust,
                                     graph and physics-informed models.
  07_data_and_storage.txt            Raw event windows, time series,
                                     geometry, analytics.
  08_connectivity_and_protocols.txt  How data reaches us: MQTT, OPC UA,
                                     Modbus, DNP3, vendor APIs, messaging.
  09_services_and_infrastructure.txt Services, observability, hosting in
                                     South Africa, POPIA, security.
  10_quality_and_verification.txt    How we prove the code is right.
  11_frontier_what_we_must_invent.txt
                                     Unsolved problems and our plan for each.
  12_decision_log.txt                Technology decisions made and pending.

How to add a technology
-----------------------
Add it to the relevant file (or a new numbered file) with:
  NAME      ring, version checked, licence, date checked
  ROLE      what it does in the engine
  WHY       why this and not the alternatives
  RISKS     maturity, licence, maintainer, lock-in
  EXIT      what we would replace it with, and how hard that is
Anything not yet confirmed against the project's own documentation or
source is marked [VERIFY], as in the rest of docs/.

Standing rules for choosing technology
--------------------------------------
T1  Licence first. Permissive (MIT / Apache-2.0 / BSD) for anything linked
    into the engine. Copyleft or commercial-only libraries are HOLD until
    a decision record accepts the cost.
T2  The physics core owns its numerics. Libraries may supply linear
    algebra, FFT and parallelism; the transient physics (MOC, boundary
    conditions, cavities, friction, viscoelasticity) is ours, because we
    must be able to test, explain and change every line (P1, P5).
T3  Determinism. Same input + same model version = bit-identical output
    on our production targets (P5: "why did the engine think this was a
    burst?" must be replayable months later).
T4  Few dependencies, each one audited. Every crate is a supply-chain risk.
T5  Nothing is chosen for fashion. A heavier technology (Kafka, Kubernetes,
    GPU) is adopted only when measured load requires it.
