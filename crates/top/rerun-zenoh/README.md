# rerun-zenoh

Native [Zenoh](https://zenoh.io) topic visualization for ROS2, built on [Rerun](https://rerun.io).

A standalone tool that connects directly to the Zenoh network to discover and visualize
ROS2 topics published via [`rmw_zenoh`](https://github.com/ros2/rmw_zenoh). **No ROS2
installation required** on the visualization machine -- no DDS, no `rclpy`, no `ament`.
Just build and point it at your Zenoh network.

## Quick start

### Build

```bash
# From the Rerun workspace root:
cargo build --release -p rerun-zenoh
```

The binary will be at `target/release/rerun-zenoh`.

### Run

```bash
# Auto-subscribe to all supported topics (launches the Rerun Viewer automatically):
rerun-zenoh --auto-subscribe

# Interactive topic selection (default):
rerun-zenoh

# Subscribe to specific topics:
rerun-zenoh -t /camera/color/image_raw -t /scan -t /tf
```

## Prerequisites

**No ROS2 installation is needed** on the machine running `rerun-zenoh`. It is a
standalone Rust binary that speaks the Zenoh protocol directly. You can visualize a
ROS2 robot from any machine that can reach the Zenoh network.

The only requirement is on the **robot side**: the ROS2 nodes publishing data must use
[`rmw_zenoh_cpp`](https://github.com/ros2/rmw_zenoh) as their ROS2 middleware:

```bash
# On the robot / ROS2 machine:
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch my_robot bringup.launch.py
```

For **cross-machine** setups (visualization laptop != robot), run a
[Zenoh router](https://zenoh.io/docs/getting-started/installation/) on one of the
machines (or a third host) so peers can discover each other:

```bash
zenohd
```

Then point `rerun-zenoh` at it:

```bash
# On your visualization machine (no ROS2 needed):
rerun-zenoh --zenoh-connect tcp/<router-ip>:7447 --auto-subscribe
```

## Usage examples

### Basic visualization

```bash
# Discover and interactively select topics on the default ROS2 domain (0):
rerun-zenoh

# Discover on a different ROS2 domain:
rerun-zenoh --domain-id 42
```

### Subscribe to specific topics

```bash
# Camera + TF + laser scan:
rerun-zenoh -t /camera/color/image_raw -t /tf -t /scan

# All supported types automatically:
rerun-zenoh --auto-subscribe
```

### Connect to a remote Zenoh router

```bash
# Connect to a specific Zenoh router:
rerun-zenoh --zenoh-connect tcp/192.168.1.100:7447 --auto-subscribe

# Use a Zenoh configuration file:
rerun-zenoh --zenoh-config my_zenoh_config.json5 --auto-subscribe
```

### Load a URDF robot model

```bash
# Load a URDF file and visualize the robot alongside sensor data:
rerun-zenoh --urdf /path/to/robot.urdf --auto-subscribe

# With a custom entity path prefix:
rerun-zenoh --urdf robot.urdf --entity-prefix my_robot --auto-subscribe
```

### Record a session

```bash
# Save to an .rrd file for later replay in Rerun Viewer:
rerun-zenoh --auto-subscribe --recording session.rrd
```

## Supported message types

| ROS2 Message Type | Rerun Archetype | Description |
|---|---|---|
| `tf2_msgs/msg/TFMessage` | `Transform3D` | Coordinate frame transforms |
| `sensor_msgs/msg/Image` | `Image` / `DepthImage` | Raw camera images |
| `sensor_msgs/msg/CompressedImage` | `EncodedImage` | JPEG/PNG compressed images |
| `sensor_msgs/msg/PointCloud2` | `Points3D` | 3D point clouds with optional colors |
| `sensor_msgs/msg/LaserScan` | `Points3D` | 2D laser scans (polar to cartesian) |
| `sensor_msgs/msg/CameraInfo` | `Pinhole` | Camera intrinsic parameters |
| `sensor_msgs/msg/Imu` | `Scalars` + `Transform3D` | Accelerometer, gyroscope, orientation |
| `geometry_msgs/msg/PoseStamped` | `Transform3D` | 6-DOF poses |
| `nav_msgs/msg/Odometry` | `Transform3D` + `Arrows3D` | Robot odometry with velocity arrows |
| `nav_msgs/msg/Path` | `LineStrips3D` | Planned/recorded paths |
| `nav_msgs/msg/OccupancyGrid` | `Image` + `Transform3D` | 2D occupancy maps |
| `visualization_msgs/msg/Marker` | Various | RViz-compatible markers |
| `visualization_msgs/msg/MarkerArray` | Various | Batched RViz markers |
| `std_msgs/msg/String` | `TextLog` | Text messages |
| `sensor_msgs/msg/Temperature` | `Scalars` | Temperature readings |
| `sensor_msgs/msg/FluidPressure` | `Scalars` | Pressure sensor data |
| `sensor_msgs/msg/RelativeHumidity` | `Scalars` | Humidity sensor data |
| `sensor_msgs/msg/Illuminance` | `Scalars` | Light sensor data |

## CLI reference

```
rerun-zenoh [OPTIONS]

Options:
      --zenoh-config <PATH>     Zenoh configuration file (JSON5 format)
      --zenoh-connect <ENDPOINT> Zenoh router endpoint (e.g., tcp/192.168.1.100:7447)
      --zenoh-listen <ENDPOINT>  Zenoh listener endpoint for peer mode
      --zenoh-mode <MODE>       Zenoh session mode: peer or client [default: peer]
      --domain-id <ID>          ROS2 domain ID [default: 0]
      --urdf <PATH>             URDF robot description file to load
      --urdf-topic <TOPIC>      ROS2 topic for dynamic robot_description
      --tf-prefix <PREFIX>      TF frame prefix for multi-robot setups
      --entity-prefix <PREFIX>  Entity path prefix in Rerun [default: ros2]
      --auto-subscribe          Auto-subscribe to all supported topic types
      --recording <PATH>        Save session to .rrd recording file
  -t, --topic <TOPIC>           Specific topic(s) to subscribe to (repeatable)
  -h, --help                    Print help
  -V, --version                 Print version
```

## Architecture

```
ROS2 Nodes (rmw_zenoh)
    |
    v  Zenoh network
+-------------------+
| re_zenoh_discovery|  Topic discovery via liveliness/admin space
+-------------------+
    |
    v  DiscoveredTopic
+--------------------+
| re_zenoh_subscriber|  Per-topic Zenoh subscribers, CDR payloads
+--------------------+
    |
    v  ZenohMessage (topic, type, payload)
+-----------------+
| re_zenoh_bridge |  CDR decode -> Rerun archetypes (14 loggers)
+-----------------+
    |
    v  RecordingStream::log()
+-----------------+
| Rerun Viewer    |  Unmodified viewer via gRPC (spawn)
+-----------------+
```

The tool connects to the Zenoh network as a peer, discovers ROS2 topics by parsing
`rmw_zenoh` key-expressions, subscribes to selected topics, deserializes CDR payloads
using the same decoder as `re_mcap`, and maps each message type to the appropriate
Rerun archetype via the `TopicLogger` trait.
