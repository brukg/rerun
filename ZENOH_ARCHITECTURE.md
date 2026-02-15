# Zenoh-ROS2 Bridge Architecture

Native Zenoh topic visualization for ROS2, built on Rerun. Connects directly to a Zenoh network to discover and visualize ROS2 topics published via `rmw_zenoh`, without requiring any DDS bridge.

## Crate overview

```
crates/
├── top/rerun-zenoh/           # CLI binary and app orchestration
├── utils/re_zenoh_discovery/  # Session management + topic discovery
├── store/re_zenoh_subscriber/ # Per-topic Zenoh subscriptions
└── store/re_zenoh_bridge/     # Message deserialization + Rerun logging
```

| Crate                  | Description                                                        |
| ---------------------- | ------------------------------------------------------------------ |
| `rerun-zenoh`          | CLI binary. Parses args, wires discovery/subscription/bridge loop. |
| `re_zenoh_discovery`   | Zenoh session, liveliness-based topic discovery, key expr parsing. |
| `re_zenoh_subscriber`  | Manages per-topic Zenoh subscribers, delivers messages via mpsc.   |
| `re_zenoh_bridge`      | Routes incoming CDR payloads to type-specific Rerun loggers.       |

## Data flow

```
rmw_zenoh (ROS2 nodes)
    │
    │  Zenoh liveliness tokens (@ros2_lv/...)
    │  Zenoh pub/sub data (<domain>/<topic>/<dds_type>/<hash>)
    ▼
re_zenoh_discovery          ──► Discovers topics via liveliness query
    │
    ▼
re_zenoh_subscriber         ──► Subscribes to data key expressions
    │
    │  CDR-encoded payloads
    ▼
re_zenoh_bridge             ──► Deserializes CDR, logs Rerun archetypes
    │
    ▼
Rerun Viewer (via re_sdk spawn/connect)
```

## rmw_zenoh key expression formats

### Liveliness tokens (discovery)

Declared by each ROS2 node under the `@ros2_lv` prefix:

```
@ros2_lv/<domain_id>/<zid>/<nid>/<id>/<entity_kind>/<enclave>/<namespace>/<node_name>/<topic_name>/<type_name>/<type_hash>/<qos>
```

| Index | Field         | Example                                    |
| ----- | ------------- | ------------------------------------------ |
| 0     | prefix        | `@ros2_lv`                                 |
| 1     | domain_id     | `0`                                        |
| 2     | zid           | `b89f34a8f6a786788926e9c41d11be8e`         |
| 3     | nid           | `1`                                        |
| 4     | id            | `5`                                        |
| 5     | entity_kind   | `MP` (pub), `MS` (sub), `NN` (node), etc. |
| 6     | enclave       | `%` (mangled `/`)                          |
| 7     | namespace     | `%prometheus` (mangled `/prometheus`)       |
| 8     | node_name     | `pekf_slam_node`                           |
| 9     | topic_name    | `%prometheus%imu` (mangled)                |
| 10    | type_name     | `sensor_msgs::msg::dds_::Imu_`            |
| 11    | type_hash     | `RIHS01_7d9a00ff...`                       |
| 12    | qos           | `::,10:,:,:,,`                             |

**Mangling**: `/` is replaced with `%` in topic names, namespaces, and enclaves.

**DDS type names**: `sensor_msgs::msg::dds_::Image_` maps to the clean ROS2 name `sensor_msgs::msg::Image` (strip `::dds_::` wrapper and trailing `_`).

**Important**: The `@` prefix is hermetic in Zenoh -- wildcards `*` and `**` never match it. Discovery must query `@ros2_lv/**` explicitly.

### Data key expressions (pub/sub)

Actual message data flows on:

```
<domain_id>/<topic_name>/<dds_type_name>/<type_hash>
```

Example: `0/prometheus/imu/sensor_msgs::msg::dds_::Imu_/RIHS01_7d9a00ff...`

## Supported message types

| ROS2 type                                | Rerun visualization          |
| ---------------------------------------- | ---------------------------- |
| `tf2_msgs::msg::TFMessage`               | Transform3D                  |
| `sensor_msgs::msg::Image`                | Image                        |
| `sensor_msgs::msg::CompressedImage`      | Image (encoded)              |
| `sensor_msgs::msg::PointCloud2`          | Points3D                     |
| `sensor_msgs::msg::LaserScan`            | Points3D (polar to cart)     |
| `sensor_msgs::msg::CameraInfo`           | Pinhole camera               |
| `sensor_msgs::msg::Imu`                  | Scalar time series           |
| `geometry_msgs::msg::PoseStamped`        | Transform3D                  |
| `nav_msgs::msg::Odometry`               | Transform3D + scalar         |
| `nav_msgs::msg::Path`                    | LineStrips3D                 |
| `nav_msgs::msg::OccupancyGrid`          | Image (grayscale)            |
| `visualization_msgs::msg::Marker`        | Various 3D primitives        |
| `visualization_msgs::msg::MarkerArray`   | Various 3D primitives        |
| `std_msgs::msg::String`                  | TextLog                      |
| `sensor_msgs::msg::Temperature`          | Scalar                       |
| `sensor_msgs::msg::FluidPressure`        | Scalar                       |
| `sensor_msgs::msg::RelativeHumidity`     | Scalar                       |
| `sensor_msgs::msg::Illuminance`          | Scalar                       |

## Usage

```bash
# Build
cargo build --release -p rerun-zenoh

# Connect to a Zenoh router and auto-subscribe to all supported topics
./target/release/rerun-zenoh \
    --zenoh-mode client \
    --zenoh-connect tcp/192.168.1.104:7447 \
    --auto-subscribe

# Subscribe to specific topics only
./target/release/rerun-zenoh \
    --zenoh-mode client \
    --zenoh-connect tcp/192.168.1.104:7447 \
    -t /prometheus/scan -t /tf

# Load a URDF alongside live data
./target/release/rerun-zenoh \
    --zenoh-mode client \
    --zenoh-connect tcp/192.168.1.104:7447 \
    --auto-subscribe \
    --urdf robot.urdf
```

## Adding support for a new message type

1. Add a CDR deserializer + Rerun logger in `re_zenoh_bridge/src/loggers/`.
2. Register it in `re_zenoh_bridge/src/logger_registry.rs` under `with_defaults()`.
3. Add the type string to `bridge_supported_types()` in `rerun-zenoh/src/app.rs`.
