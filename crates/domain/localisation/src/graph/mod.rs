//! The wave-travel graph: how long a pressure wave takes to get anywhere.
//!
//! `burst_localisation.txt` section 2: the first arrival at a sensor comes
//! along the FASTEST path, so travel times are shortest paths with edge
//! weights tau = L / a (Dijkstra).
//!
//! One physics rule shapes the graph: a wave does not pass THROUGH a
//! reservoir. A reservoir holds its head fixed, so it reflects a wave fully
//! (R = -1) and transmits nothing (`joukowsky_equation.txt` section 6).
//! Paths may end at a reservoir but never continue beyond it. Junctions,
//! check valves and pumps are passed through (first-arrival approximation;
//! the amplitude lost at each is ignored here).

pub mod travel_time;

pub(crate) use travel_time::Routes;
pub use travel_time::TravelGraph;
