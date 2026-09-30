//! The wave-travel graph: how long a pressure wave takes to get anywhere.
//!
//! `burst_localisation.txt` section 2: the first arrival at a sensor comes
//! along the FASTEST path, so travel times are shortest paths with edge
//! weights tau = L / a (Dijkstra).
//!
//! One physics rule shapes the graph: a wave does not pass THROUGH a
//! reservoir. A reservoir holds its head fixed, so it reflects a wave fully
//! (R = -1) and transmits nothing (`joukowsky_equation.txt` section 6).
//! Paths may end at a reservoir but never continue beyond it. The same holds
//! for devices the steady state leaves SHUT - a check valve or PRV held
//! shut, a pump whose non-return valve is shut ([`devices`], FN-029).
//! Junctions and open devices are passed through (the amplitude lost at
//! each matters for whether a wave is felt - placement/detectability - not
//! for when it arrives).

pub mod devices;
pub mod travel_time;

pub use devices::Devices;
pub(crate) use travel_time::Routes;
pub use travel_time::TravelGraph;
