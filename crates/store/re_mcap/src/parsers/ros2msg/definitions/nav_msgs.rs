//! Definitions for the ROS2 `nav_msgs` package.
//!
//! Based on definitions taken from <https://github.com/ros2/common_interfaces/tree/rolling/nav_msgs>

use serde::{Deserialize, Serialize};

use super::builtin_interfaces::Time;
use super::geometry_msgs::{Pose, PoseStamped, PoseWithCovariance, TwistWithCovariance};
use super::std_msgs::Header;

/// Metadata about the map.
#[derive(Debug, Serialize, Deserialize)]
pub struct MapMetaData {
    /// The time at which the map was loaded.
    pub map_load_time: Time,

    /// The map resolution (meters/cell).
    pub resolution: f32,

    /// Map width (cells).
    pub width: u32,

    /// Map height (cells).
    pub height: u32,

    /// The origin of the map. This is the real-world pose of the cell (0,0) in the map.
    pub origin: Pose,
}

/// Represents a 2D grid map, in which each cell represents the probability of occupancy.
#[derive(Debug, Serialize, Deserialize)]
pub struct OccupancyGrid {
    pub header: Header,

    /// MetaData for the map.
    pub info: MapMetaData,

    /// The map data, in row-major order, starting with (0,0).
    /// Occupancy probabilities are in the range [0,100]. Unknown is -1.
    pub data: Vec<i8>,
}

/// Represents an estimate of a position and velocity in free space.
#[derive(Debug, Serialize, Deserialize)]
pub struct Odometry {
    pub header: Header,

    /// Frame id the pose points to.
    pub child_frame_id: String,

    /// Estimated pose that is typically relative to a fixed world frame.
    pub pose: PoseWithCovariance,

    /// Estimated linear and angular velocity relative to child_frame_id.
    pub twist: TwistWithCovariance,
}

/// An array of poses that represents a path for a robot to follow.
#[derive(Debug, Serialize, Deserialize)]
pub struct Path {
    pub header: Header,

    /// The poses along the path.
    pub poses: Vec<PoseStamped>,
}
