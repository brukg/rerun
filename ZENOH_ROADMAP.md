# Zenoh-ROS2 Bridge Roadmap

Current status: **MVP -- live topic discovery and visualization working over rmw_zenoh.**

## Completed

- [x] Zenoh session management (peer and client modes)
- [x] Topic discovery via `@ros2_lv` liveliness tokens
- [x] rmw_zenoh key expression parsing (mangling, DDS type conversion)
- [x] Per-topic Zenoh subscription with correct data key expressions
- [x] CDR deserialization for 18 common ROS2 message types
- [x] Live streaming to Rerun viewer via `re_sdk` spawn/connect
- [x] URDF loading for robot model visualization
- [x] Auto-subscribe mode for hands-free operation
- [x] Interactive topic selection CLI

## Short term

- [ ] Continuous discovery -- subscribe to `@ros2_lv/**` liveliness changes so new topics are picked up without restarting
- [ ] Reconnection handling -- recover gracefully when the Zenoh router or ROS2 nodes restart
- [ ] QoS-aware subscriptions -- parse the QoS field from liveliness tokens and configure subscriber reliability/durability accordingly
- [ ] `geometry_msgs::msg::PoseWithCovarianceStamped` support (common in Nav2 stacks)
- [ ] `sensor_msgs::msg::JointState` support with URDF-driven transform tree
- [ ] Service introspection (list available ROS2 services via `SS`/`SC` liveliness tokens)

## Medium term

- [ ] Zero-copy shared memory transport (`transport_shm` feature) for same-host deployments
- [ ] Multi-domain support -- discover and visualize topics across multiple ROS2 domain IDs simultaneously
- [ ] Web viewer integration -- stream data to `rerun --web-viewer` for browser-based visualization
- [ ] Recording to `.rrd` file while streaming (the `--recording` flag plumbing exists but is not wired)
- [ ] TF tree visualization -- full transform tree from `/tf` and `/tf_static`, with frame graph display
- [ ] Namespace-aware entity mapping -- group entities by ROS2 namespace in the Rerun blueprint

## Longer term

- [ ] Action introspection -- visualize Nav2 action goals, feedback, and status
- [ ] Dynamic URDF via `/robot_description` topic subscription
- [ ] ROS2 parameter inspection and live tuning through Rerun UI
- [ ] Configurable message type plugins -- load custom CDR deserializers at runtime
- [ ] Performance benchmarks and optimization for high-bandwidth topics (e.g. 30fps cameras, dense point clouds)
- [ ] Integration with Rerun blueprints -- auto-generate spatial view layouts from TF tree structure
