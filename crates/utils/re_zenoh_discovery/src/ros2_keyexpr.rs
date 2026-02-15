//! Parser for ROS2 key-expressions as used by `rmw_zenoh`.
//!
//! The `rmw_zenoh` middleware declares liveliness tokens with the format:
//! ```text
//! @ros2_lv/<domain_id>/<zid>/<nid>/<id>/<entity_kind>/<enclave>/<namespace>/<node_name>/<topic_name>/<type_name>/<type_hash>/<qos>
//! ```
//!
//! Where topic names are "mangled" by replacing `/` with `%`, and type names
//! use DDS-style naming (e.g., `sensor_msgs::msg::dds_::Image_`).
//!
//! The actual data key-expression for subscribing to topic data is:
//! ```text
//! <domain_id>/<topic_name>/<type_name>/<type_hash>
//! ```

/// The rmw_zenoh liveliness key prefix.
pub const LIVELINESS_PREFIX: &str = "@ros2_lv";

/// The kind of ROS2 endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EndpointKind {
    Publisher,
    Subscriber,
}

impl std::fmt::Display for EndpointKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Publisher => write!(f, "pub"),
            Self::Subscriber => write!(f, "sub"),
        }
    }
}

/// A parsed ROS2 key-expression from `rmw_zenoh`.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Ros2KeyExpr {
    /// The ROS2 domain ID (typically 0).
    pub domain_id: u32,

    /// The ROS2 topic name (e.g., `/camera/color/image_raw`).
    pub topic_name: String,

    /// The ROS2 message type in clean format (e.g., `sensor_msgs::msg::Image`).
    pub type_name: String,

    /// Whether this is a publisher or subscriber endpoint.
    pub endpoint_kind: EndpointKind,

    /// The original full key-expression string.
    pub full_key_expr: String,

    /// The data key expression for subscribing to actual topic data.
    pub data_key_expr: String,
}

/// Replace `%` with `/` to reverse rmw_zenoh name mangling.
fn demangle_name(mangled: &str) -> String {
    mangled.replace('%', "/")
}

/// Convert DDS-style type name to clean ROS2 style.
///
/// `sensor_msgs::msg::dds_::Image_` becomes `sensor_msgs::msg::Image`
fn dds_type_to_ros2(dds_type: &str) -> String {
    if dds_type.contains("::dds_::") {
        let result = dds_type.replace("::dds_::", "::");
        result.strip_suffix('_').unwrap_or(&result).to_owned()
    } else {
        dds_type.to_owned()
    }
}

impl Ros2KeyExpr {
    /// Parse a key-expression string from `rmw_zenoh`.
    ///
    /// Supports two formats:
    /// 1. rmw_zenoh liveliness tokens: `@ros2_lv/<domain>/<zid>/<nid>/<id>/<kind>/<enc>/<ns>/<node>/<topic>/<type>/<hash>/<qos>`
    /// 2. Legacy simple format: `<domain>/<topic>/<type>/<pub|sub>`
    pub fn parse(key_expr: &str) -> Option<Self> {
        let parts: Vec<&str> = key_expr.split('/').collect();

        if parts.first().is_some_and(|p| *p == LIVELINESS_PREFIX) {
            Self::parse_liveliness(&parts, key_expr)
        } else {
            Self::parse_legacy(&parts, key_expr)
        }
    }

    /// Parse the rmw_zenoh liveliness token format.
    ///
    /// Indices:
    /// - 0: `@ros2_lv`
    /// - 1: domain_id
    /// - 2: zid (zenoh session ID)
    /// - 3: nid (node ID)
    /// - 4: id (entity ID)
    /// - 5: entity kind (`MP`, `MS`, `SS`, `SC`, `NN`)
    /// - 6: enclave (mangled)
    /// - 7: namespace (mangled)
    /// - 8: node name
    /// - 9: topic name (mangled)
    /// - 10: type name (DDS-style)
    /// - 11: type hash
    /// - 12: qos
    fn parse_liveliness(parts: &[&str], key_expr: &str) -> Option<Self> {
        // Publishers/subscribers need at least 13 parts
        if parts.len() < 13 {
            return None;
        }

        let domain_id: u32 = parts[1].parse().ok()?;

        let endpoint_kind = match parts[5] {
            "MP" => EndpointKind::Publisher,
            "MS" => EndpointKind::Subscriber,
            _ => return None, // Skip nodes (NN), services (SS, SC)
        };

        let topic_name_mangled = parts[9];
        let dds_type_name = parts[10];
        let type_hash = parts[11];

        let topic_name = demangle_name(topic_name_mangled);
        let type_name = dds_type_to_ros2(dds_type_name);

        let data_key_expr = format!(
            "{}/{}/{}/{}",
            domain_id,
            topic_name.trim_start_matches('/'),
            dds_type_name,
            type_hash
        );

        Some(Self {
            domain_id,
            topic_name,
            type_name,
            endpoint_kind,
            full_key_expr: key_expr.to_owned(),
            data_key_expr,
        })
    }

    /// Parse the legacy simple format: `<domain_id>/<topic>/<type>/<pub|sub>`.
    fn parse_legacy(parts: &[&str], key_expr: &str) -> Option<Self> {
        if parts.len() < 4 {
            return None;
        }

        let endpoint_kind = match *parts.last()? {
            "pub" => EndpointKind::Publisher,
            "sub" => EndpointKind::Subscriber,
            _ => return None,
        };

        let type_name = parts[parts.len() - 2];
        if !type_name.contains("::") {
            return None;
        }

        let domain_id: u32 = parts[0].parse().ok()?;

        let topic_parts = &parts[1..parts.len() - 2];
        if topic_parts.is_empty() {
            return None;
        }
        let topic_name = format!("/{}", topic_parts.join("/"));

        let data_key_expr = format!(
            "{}/{}/{}",
            domain_id,
            topic_name.trim_start_matches('/'),
            type_name
        );

        Some(Self {
            domain_id,
            topic_name,
            type_name: type_name.to_owned(),
            endpoint_kind,
            full_key_expr: key_expr.to_owned(),
            data_key_expr,
        })
    }

    /// Returns the message type in ROS2 format (e.g., `sensor_msgs/msg/Image`).
    pub fn type_name_ros2(&self) -> String {
        self.type_name.replace("::", "/")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- rmw_zenoh liveliness format tests ---

    #[test]
    fn test_parse_liveliness_publisher() {
        let key = "@ros2_lv/0/b89f34a8f6a786788926e9c41d11be8e/1/5/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,,";
        let parsed = Ros2KeyExpr::parse(key).expect("should parse");
        assert_eq!(parsed.domain_id, 0);
        assert_eq!(parsed.topic_name, "/chatter");
        assert_eq!(parsed.type_name, "std_msgs::msg::String");
        assert_eq!(parsed.endpoint_kind, EndpointKind::Publisher);
        assert_eq!(
            parsed.data_key_expr,
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_abc123"
        );
    }

    #[test]
    fn test_parse_liveliness_subscriber() {
        let key = "@ros2_lv/2/aac3178e146ba6f1fc6e6a4085e77f21/0/10/MS/%/%/listener/%chatter/std_msgs::msg::dds_::String_/RIHS01_abc123/::,10:,:,:,,";
        let parsed = Ros2KeyExpr::parse(key).expect("should parse");
        assert_eq!(parsed.domain_id, 2);
        assert_eq!(parsed.topic_name, "/chatter");
        assert_eq!(parsed.type_name, "std_msgs::msg::String");
        assert_eq!(parsed.endpoint_kind, EndpointKind::Subscriber);
    }

    #[test]
    fn test_parse_liveliness_deep_topic() {
        let key = "@ros2_lv/0/b89f/1/5/MP/%/%/cam_node/%camera%color%image_raw/sensor_msgs::msg::dds_::Image_/RIHS01_xyz/::,10:,:,:,,";
        let parsed = Ros2KeyExpr::parse(key).expect("should parse");
        assert_eq!(parsed.topic_name, "/camera/color/image_raw");
        assert_eq!(parsed.type_name, "sensor_msgs::msg::Image");
        assert_eq!(
            parsed.data_key_expr,
            "0/camera/color/image_raw/sensor_msgs::msg::dds_::Image_/RIHS01_xyz"
        );
    }

    #[test]
    fn test_parse_liveliness_node_skipped() {
        let key = "@ros2_lv/0/b89f/1/1/NN/%/%/talker";
        assert!(Ros2KeyExpr::parse(key).is_none());
    }

    #[test]
    fn test_parse_liveliness_service_skipped() {
        let key = "@ros2_lv/0/b89f/0/10/SS/%/%/server/%add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/RIHS01_abc/::,10:,:,:,,";
        assert!(Ros2KeyExpr::parse(key).is_none());
    }

    // --- DDS type conversion tests ---

    #[test]
    fn test_dds_type_to_ros2() {
        assert_eq!(
            dds_type_to_ros2("sensor_msgs::msg::dds_::Image_"),
            "sensor_msgs::msg::Image"
        );
        assert_eq!(
            dds_type_to_ros2("std_msgs::msg::dds_::String_"),
            "std_msgs::msg::String"
        );
        assert_eq!(
            dds_type_to_ros2("tf2_msgs::msg::dds_::TFMessage_"),
            "tf2_msgs::msg::TFMessage"
        );
        // Already clean types pass through
        assert_eq!(
            dds_type_to_ros2("sensor_msgs::msg::Image"),
            "sensor_msgs::msg::Image"
        );
    }

    // --- Demangling tests ---

    #[test]
    fn test_demangle_name() {
        assert_eq!(demangle_name("%chatter"), "/chatter");
        assert_eq!(
            demangle_name("%camera%color%image_raw"),
            "/camera/color/image_raw"
        );
        assert_eq!(demangle_name("%"), "/");
        assert_eq!(demangle_name("no_slashes"), "no_slashes");
    }

    // --- Legacy format tests ---

    #[test]
    fn test_parse_legacy_basic() {
        let parsed = Ros2KeyExpr::parse("0/camera/color/image_raw/sensor_msgs::msg::Image/pub")
            .expect("should parse");
        assert_eq!(parsed.domain_id, 0);
        assert_eq!(parsed.topic_name, "/camera/color/image_raw");
        assert_eq!(parsed.type_name, "sensor_msgs::msg::Image");
        assert_eq!(parsed.endpoint_kind, EndpointKind::Publisher);
    }

    #[test]
    fn test_parse_legacy_tf() {
        let parsed =
            Ros2KeyExpr::parse("0/tf/tf2_msgs::msg::TFMessage/pub").expect("should parse");
        assert_eq!(parsed.domain_id, 0);
        assert_eq!(parsed.topic_name, "/tf");
        assert_eq!(parsed.type_name, "tf2_msgs::msg::TFMessage");
        assert_eq!(parsed.endpoint_kind, EndpointKind::Publisher);
    }

    #[test]
    fn test_parse_legacy_subscriber() {
        let parsed =
            Ros2KeyExpr::parse("0/cmd_vel/geometry_msgs::msg::Twist/sub").expect("should parse");
        assert_eq!(parsed.domain_id, 0);
        assert_eq!(parsed.topic_name, "/cmd_vel");
        assert_eq!(parsed.type_name, "geometry_msgs::msg::Twist");
        assert_eq!(parsed.endpoint_kind, EndpointKind::Subscriber);
    }

    #[test]
    fn test_parse_legacy_domain_id() {
        let parsed = Ros2KeyExpr::parse("42/scan/sensor_msgs::msg::LaserScan/pub")
            .expect("should parse");
        assert_eq!(parsed.domain_id, 42);
        assert_eq!(parsed.topic_name, "/scan");
    }

    #[test]
    fn test_parse_legacy_deep_topic() {
        let parsed = Ros2KeyExpr::parse(
            "0/robot1/arm/joint_states/sensor_msgs::msg::JointState/pub",
        )
        .expect("should parse");
        assert_eq!(parsed.topic_name, "/robot1/arm/joint_states");
        assert_eq!(parsed.type_name, "sensor_msgs::msg::JointState");
    }

    #[test]
    fn test_parse_invalid() {
        assert!(Ros2KeyExpr::parse("").is_none());
        assert!(Ros2KeyExpr::parse("0/topic/pub").is_none()); // missing type
        assert!(Ros2KeyExpr::parse("0/topic/String/pub").is_none()); // type without ::
        assert!(Ros2KeyExpr::parse("not_a_number/topic/std_msgs::msg::String/pub").is_none());
    }

    #[test]
    fn test_type_name_ros2() {
        let parsed = Ros2KeyExpr::parse("0/tf/tf2_msgs::msg::TFMessage/pub").unwrap();
        assert_eq!(parsed.type_name_ros2(), "tf2_msgs/msg/TFMessage");
    }
}
