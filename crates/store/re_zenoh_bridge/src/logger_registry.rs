//! Topic logger trait and registry for mapping ROS2 message types to loggers.

use std::collections::HashMap;

use re_sdk::RecordingStream;

/// Trait for logging a single ROS2 message to a Rerun recording stream.
///
/// Unlike the MCAP batch parsers, this processes one message at a time
/// for live streaming from Zenoh.
pub trait TopicLogger: Send {
    /// Process a single CDR-encoded message and log it via the `RecordingStream`.
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()>;

    /// Return the ROS2 type name this logger handles.
    fn type_name(&self) -> &str;
}

type LoggerFactory = Box<dyn Fn() -> Box<dyn TopicLogger> + Send + Sync>;

/// Registry mapping ROS2 message type names to logger factories.
pub struct LoggerRegistry {
    factories: HashMap<String, LoggerFactory>,
}

impl LoggerRegistry {
    /// Create an empty registry.
    pub fn new() -> Self {
        Self {
            factories: HashMap::new(),
        }
    }

    /// Create a registry pre-populated with all supported ROS2 message types.
    pub fn with_defaults() -> Self {
        use crate::loggers;

        let mut registry = Self::new();

        // TF
        registry.register("tf2_msgs::msg::TFMessage", || {
            Box::new(loggers::tf::TfLogger)
        });

        // Images
        registry.register("sensor_msgs::msg::Image", || {
            Box::new(loggers::image::ImageLogger)
        });
        registry.register("sensor_msgs::msg::CompressedImage", || {
            Box::new(loggers::compressed_image::CompressedImageLogger)
        });

        // Spatial
        registry.register("sensor_msgs::msg::PointCloud2", || {
            Box::new(loggers::pointcloud::PointCloudLogger)
        });
        registry.register("sensor_msgs::msg::LaserScan", || {
            Box::new(loggers::laser_scan::LaserScanLogger)
        });

        // Camera
        registry.register("sensor_msgs::msg::CameraInfo", || {
            Box::new(loggers::camera_info::CameraInfoLogger)
        });

        // Poses and transforms
        registry.register("geometry_msgs::msg::PoseStamped", || {
            Box::new(loggers::pose::PoseLogger)
        });

        // Navigation
        registry.register("nav_msgs::msg::Odometry", || {
            Box::new(loggers::odometry::OdometryLogger)
        });
        registry.register("nav_msgs::msg::Path", || {
            Box::new(loggers::path::PathLogger)
        });
        registry.register("nav_msgs::msg::OccupancyGrid", || {
            Box::new(loggers::occupancy_grid::OccupancyGridLogger)
        });

        // IMU
        registry.register("sensor_msgs::msg::Imu", || {
            Box::new(loggers::imu::ImuLogger)
        });

        // Markers
        registry.register("visualization_msgs::msg::Marker", || {
            Box::new(loggers::marker::MarkerLogger)
        });
        registry.register("visualization_msgs::msg::MarkerArray", || {
            Box::new(loggers::marker::MarkerArrayLogger)
        });

        // Text
        registry.register("std_msgs::msg::String", || {
            Box::new(loggers::string::StringLogger)
        });

        // Scalar types
        for type_name in [
            "sensor_msgs::msg::Temperature",
            "sensor_msgs::msg::FluidPressure",
            "sensor_msgs::msg::RelativeHumidity",
            "sensor_msgs::msg::Illuminance",
        ] {
            let tn = type_name.to_owned();
            registry.register(type_name, move || {
                Box::new(loggers::scalar::ScalarLogger::new(tn.clone()))
            });
        }

        registry
    }

    /// Register a logger factory for a given ROS2 message type name.
    pub fn register<F>(&mut self, type_name: &str, factory: F)
    where
        F: Fn() -> Box<dyn TopicLogger> + Send + Sync + 'static,
    {
        self.factories
            .insert(type_name.to_owned(), Box::new(factory));
    }

    /// Create a logger for a given ROS2 message type, if supported.
    pub fn create_logger(&self, type_name: &str) -> Option<Box<dyn TopicLogger>> {
        self.factories.get(type_name).map(|factory| factory())
    }

    /// Check if a message type is supported.
    pub fn is_supported(&self, type_name: &str) -> bool {
        self.factories.contains_key(type_name)
    }

    /// Get all supported type names.
    pub fn supported_types(&self) -> Vec<&str> {
        self.factories.keys().map(|s| s.as_str()).collect()
    }
}

impl Default for LoggerRegistry {
    fn default() -> Self {
        Self::with_defaults()
    }
}
