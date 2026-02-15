//! Command-line interface for the rerun-zenoh viewer.

use clap::Parser;

/// Native Zenoh topic visualization for ROS2, built on Rerun.
///
/// Connects directly to a Zenoh network to discover and visualize ROS2 topics
/// published via `rmw_zenoh`, without requiring any DDS bridge.
#[derive(Parser, Debug)]
#[command(name = "rerun-zenoh", version, about)]
pub struct Cli {
    /// Path to a Zenoh configuration file (JSON5 format).
    #[arg(long)]
    pub zenoh_config: Option<String>,

    /// Zenoh router endpoint to connect to (e.g., `tcp/192.168.1.100:7447`).
    #[arg(long)]
    pub zenoh_connect: Option<String>,

    /// Zenoh listener endpoint for peer mode (e.g., `tcp/0.0.0.0:7447`).
    #[arg(long)]
    pub zenoh_listen: Option<String>,

    /// Zenoh session mode: `peer` or `client`.
    #[arg(long, default_value = "peer")]
    pub zenoh_mode: String,

    /// ROS2 domain ID for topic discovery.
    #[arg(long, default_value = "0")]
    pub domain_id: u32,

    /// Load a URDF robot description file on startup.
    #[arg(long)]
    pub urdf: Option<String>,

    /// ROS2 topic for dynamic `robot_description` (URDF).
    #[arg(long)]
    pub urdf_topic: Option<String>,

    /// TF frame prefix for multi-robot setups.
    #[arg(long)]
    pub tf_prefix: Option<String>,

    /// Entity path prefix for all ROS2 data in Rerun.
    #[arg(long, default_value = "ros2")]
    pub entity_prefix: String,

    /// Automatically subscribe to all topics with supported message types.
    #[arg(long)]
    pub auto_subscribe: bool,

    /// Save session to a Rerun .rrd recording file.
    #[arg(long)]
    pub recording: Option<String>,

    /// Specific topics to subscribe to (can be repeated).
    #[arg(long, short = 't')]
    pub topic: Vec<String>,
}
