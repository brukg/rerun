//! Message type representing data received from a Zenoh subscriber.

/// A message received from a Zenoh topic subscription.
#[derive(Debug, Clone)]
pub struct ZenohMessage {
    /// The ROS2 topic name (e.g., `/camera/color/image_raw`).
    pub topic_name: String,

    /// The ROS2 message type name (e.g., `sensor_msgs::msg::Image`).
    pub type_name: String,

    /// The raw CDR-encoded payload bytes.
    pub payload: Vec<u8>,

    /// Wall-clock receive time in nanoseconds since Unix epoch.
    pub receive_time_ns: u64,
}
