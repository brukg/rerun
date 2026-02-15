//! Maps ROS2 topic names to Rerun entity paths.

/// Maps ROS2 topic names to Rerun entity paths.
///
/// For example, with prefix `"ros2"`:
/// - `/camera/color/image_raw` → `ros2/camera/color/image_raw`
/// - `/tf` → `ros2/tf`
pub struct EntityMapper {
    prefix: String,
}

impl EntityMapper {
    /// Create a new entity mapper with the given prefix.
    pub fn new(prefix: &str) -> Self {
        Self {
            prefix: prefix.trim_matches('/').to_owned(),
        }
    }

    /// Map a ROS2 topic name to a Rerun entity path string.
    pub fn map_topic(&self, topic_name: &str) -> String {
        let topic = topic_name.trim_start_matches('/');
        if self.prefix.is_empty() {
            topic.to_owned()
        } else {
            format!("{}/{}", self.prefix, topic)
        }
    }

    /// Map a ROS2 topic name with a sub-path (e.g., for IMU axes).
    pub fn map_topic_sub(&self, topic_name: &str, sub_path: &str) -> String {
        let base = self.map_topic(topic_name);
        format!("{base}/{sub_path}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_mapping() {
        let mapper = EntityMapper::new("ros2");
        assert_eq!(
            mapper.map_topic("/camera/color/image_raw"),
            "ros2/camera/color/image_raw"
        );
        assert_eq!(mapper.map_topic("/tf"), "ros2/tf");
        assert_eq!(mapper.map_topic("/scan"), "ros2/scan");
    }

    #[test]
    fn test_empty_prefix() {
        let mapper = EntityMapper::new("");
        assert_eq!(mapper.map_topic("/tf"), "tf");
        assert_eq!(
            mapper.map_topic("/camera/image"),
            "camera/image"
        );
    }

    #[test]
    fn test_sub_path() {
        let mapper = EntityMapper::new("ros2");
        assert_eq!(
            mapper.map_topic_sub("/imu", "linear_acceleration/x"),
            "ros2/imu/linear_acceleration/x"
        );
    }
}
