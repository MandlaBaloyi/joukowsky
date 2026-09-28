//! Application layer: the event pipeline.
//!
//! `docs/06_engine_architecture/engine_overview.txt` ("CORE (event
//! pipeline)") and `project_structure_and_ddd.txt` (rule R3): the
//! application layer ORCHESTRATES the domain contexts - it holds no physics
//! of its own - and declares PORTS for the outside world. Adapters (SMS,
//! email, dashboards, databases) implement those ports; the core never
//! depends on them.
//!
//! One pass of the pipeline over a window of sensor records:
//!   1. detection at every sensor;
//!   2. fusion: detections at different sensors are one event only if their
//!      time differences are physically possible (Layer 4 coincidence gate);
//!   3. clocks repaired, the event located (2+ sensors) and classified;
//!   4. one alert per event, with a level, reasons and uncertainty, handed
//!      to an [`ports::AlertSink`].
//!
//! Folders:
//! - [`pipeline`]: the site description, fusion, and the use case itself.
//! - [`alert`]: what comes out.
//! - [`ports`]: the traits adapters implement.

pub mod alert;
mod error;
pub mod pipeline;
pub mod ports;

pub use alert::{Alert, Level};
pub use error::ApplicationError;
pub use pipeline::{PipelineConfig, Record, SensorSite, Site, process};
pub use ports::{AlertSink, CollectingSink};
