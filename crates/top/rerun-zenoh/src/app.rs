//! Application lifecycle management.

use std::sync::Arc;

use re_zenoh_bridge::ZenohBridge;
use re_zenoh_discovery::{TopicDiscovery, ZenohSession};
use re_zenoh_subscriber::SubscriptionManager;

use crate::cli::Cli;
use crate::topic_selector;

/// Run the rerun-zenoh application.
pub async fn run(cli: Cli) -> anyhow::Result<()> {
    // Build Zenoh configuration
    let mut zenoh_config = if let Some(config_path) = &cli.zenoh_config {
        zenoh::Config::from_file(config_path)
            .map_err(|e| anyhow::anyhow!("Failed to load Zenoh config from {config_path}: {e}"))?
    } else {
        zenoh::Config::default()
    };

    if let Some(endpoint) = &cli.zenoh_connect {
        zenoh_config
            .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
            .map_err(|e| anyhow::anyhow!("Failed to set Zenoh connect endpoint: {e}"))?;
    }

    if let Some(endpoint) = &cli.zenoh_listen {
        zenoh_config
            .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
            .map_err(|e| anyhow::anyhow!("Failed to set Zenoh listen endpoint: {e}"))?;
    }

    // Apply the session mode (peer or client)
    zenoh_config
        .insert_json5("mode", &format!("\"{}\"", cli.zenoh_mode))
        .map_err(|e| anyhow::anyhow!("Failed to set Zenoh mode: {e}"))?;

    // Connect to Zenoh
    let session = Arc::new(ZenohSession::connect(zenoh_config).await?);

    // Discover topics
    let discovery = TopicDiscovery::new(Arc::clone(&session), cli.domain_id);

    println!("Scanning for ROS2 topics on Zenoh network (domain {})...", cli.domain_id);

    // Try liveliness first, then fall back to admin space
    if let Err(e) = discovery.scan_liveliness().await {
        re_log::debug!("Liveliness scan failed ({e}), trying admin space scan");
        discovery.scan().await?;
    }

    let topics = discovery.topics().await;

    if topics.is_empty() {
        println!("No ROS2 topics found. Make sure your ROS2 nodes are running with rmw_zenoh.");
        println!("Waiting for topics to appear...");

        // Retry a few times
        for _ in 0..5 {
            tokio::time::sleep(std::time::Duration::from_secs(2)).await;
            discovery.scan_liveliness().await.ok();
            discovery.scan().await.ok();

            let topics = discovery.topics().await;
            if !topics.is_empty() {
                break;
            }
        }
    }

    let topics = discovery.topics().await;

    // Print discovered topics
    if topics.is_empty() {
        println!("No ROS2 topics found after retries.");
    } else {
        println!("Found {} topics:", topics.len());
        for t in &topics {
            println!("  {} ({})", t.topic_name, t.type_name);
        }
    }

    // Create the recording stream and spawn the viewer
    println!("Spawning Rerun viewer\u{2026}");
    let rec = rerun::RecordingStreamBuilder::new("rerun_zenoh")
        .spawn()
        .map_err(|e| anyhow::anyhow!("Failed to spawn Rerun viewer: {e}"))?;

    // Also save to file if requested
    if let Some(recording_path) = &cli.recording {
        re_log::info!("Recording to {recording_path}");
    }

    // Create the bridge
    let mut bridge = ZenohBridge::new(rec, &cli.entity_prefix);

    // Create subscription manager
    let mut sub_manager = SubscriptionManager::new(Arc::clone(&session));

    // Select topics to subscribe to
    let selected_indices = if cli.auto_subscribe {
        let supported = bridge_supported_types();
        topic_selector::select_all_supported(&topics, &supported)
    } else if !cli.topic.is_empty() {
        // Subscribe to explicitly specified topics
        let mut indices = Vec::new();
        for topic_name in &cli.topic {
            if let Some(idx) = topics.iter().position(|t| t.topic_name == *topic_name) {
                indices.push(idx);
            } else {
                re_log::warn!("Requested topic {topic_name} not found in discovery");
            }
        }
        indices
    } else {
        let supported = bridge_supported_types();
        topic_selector::select_topics_interactive(&topics, &supported)
    };

    if selected_indices.is_empty() {
        println!("No topics selected. Exiting.");
        return Ok(());
    }

    // Subscribe to selected topics
    for &idx in &selected_indices {
        let topic = &topics[idx];
        bridge.add_topic(&topic.topic_name, &topic.type_name);
        sub_manager.subscribe(topic)?;
    }

    println!(
        "\nSubscribed to {} topics. Streaming to Rerun viewer...",
        selected_indices.len()
    );
    println!("Press Ctrl+C to stop.\n");

    // Load URDF if requested
    if let Some(urdf_path) = &cli.urdf {
        load_urdf(&bridge, urdf_path, &cli.entity_prefix)?;
    }

    // Main bridge processing loop
    let mut rx = sub_manager
        .take_message_receiver()
        .expect("Message receiver should be available");

    let mut msg_count: u64 = 0;
    let start_time = std::time::Instant::now();

    loop {
        tokio::select! {
            Some(msg) = rx.recv() => {
                if let Err(e) = bridge.process_message(&msg) {
                    re_log::warn_once!(
                        "Failed to process message on {}: {e}",
                        msg.topic_name
                    );
                }
                msg_count += 1;

                // Periodic status
                if msg_count.is_multiple_of(1000) {
                    let elapsed = start_time.elapsed().as_secs_f64();
                    let rate = msg_count as f64 / elapsed;
                    re_log::info!("Processed {msg_count} messages ({rate:.0} msgs/sec)");
                }
            }
            _ = tokio::signal::ctrl_c() => {
                println!("\nShutting down...");
                break;
            }
        }
    }

    // Cleanup
    sub_manager.unsubscribe_all();
    let elapsed = start_time.elapsed().as_secs_f64();
    println!(
        "Processed {msg_count} messages in {elapsed:.1}s ({:.0} msgs/sec)",
        msg_count as f64 / elapsed
    );

    Ok(())
}

/// Load a URDF file and log the robot model to Rerun.
fn load_urdf(
    bridge: &ZenohBridge,
    urdf_path: &str,
    entity_prefix: &str,
) -> anyhow::Result<()> {
    re_log::info!("Loading URDF from {urdf_path}");

    let urdf_content = std::fs::read_to_string(urdf_path)
        .map_err(|e| anyhow::anyhow!("Failed to read URDF file {urdf_path}: {e}"))?;

    let robot = urdf_rs::read_from_string(&urdf_content)
        .map_err(|e| anyhow::anyhow!("Failed to parse URDF: {e}"))?;

    let rec = bridge.recording_stream();

    // Log each link's visual geometry
    for link in &robot.links {
        let link_path = format!("{entity_prefix}/urdf/{}", link.name);

        for visual in &link.visual {
            match &visual.geometry {
                urdf_rs::Geometry::Box { size } => {
                    rec.log_static(
                        link_path.as_str(),
                        &rerun::archetypes::Boxes3D::from_half_sizes([[
                            (size[0] / 2.0) as f32,
                            (size[1] / 2.0) as f32,
                            (size[2] / 2.0) as f32,
                        ]]),
                    )?;
                }
                urdf_rs::Geometry::Cylinder { radius, length } => {
                    rec.log_static(
                        link_path.as_str(),
                        &rerun::archetypes::Ellipsoids3D::from_half_sizes([[
                            *radius as f32,
                            *radius as f32,
                            (*length / 2.0) as f32,
                        ]]),
                    )?;
                }
                urdf_rs::Geometry::Sphere { radius } => {
                    rec.log_static(
                        link_path.as_str(),
                        &rerun::archetypes::Ellipsoids3D::from_half_sizes([[
                            *radius as f32,
                            *radius as f32,
                            *radius as f32,
                        ]]),
                    )?;
                }
                urdf_rs::Geometry::Mesh { filename, scale: _ } => {
                    re_log::info!("URDF mesh reference: {filename} (mesh loading not yet implemented)");
                }
                urdf_rs::Geometry::Capsule { .. } => {}
            }

            // Log the visual's origin transform
            let origin = &visual.origin;
            let xyz = &origin.xyz;

            rec.log_static(
                format!("{link_path}/origin").as_str(),
                &rerun::archetypes::Transform3D::update_fields()
                    .with_translation(rerun::components::Translation3D::new(
                        xyz[0] as f32,
                        xyz[1] as f32,
                        xyz[2] as f32,
                    )),
            )?;
        }
    }

    // Log joint hierarchy as transforms
    for joint in &robot.joints {
        let joint_path = format!("{entity_prefix}/urdf/{}", joint.child.link);

        let xyz = &joint.origin.xyz;

        rec.log_static(
            joint_path.as_str(),
            &rerun::archetypes::Transform3D::update_fields()
                .with_translation(rerun::components::Translation3D::new(
                    xyz[0] as f32,
                    xyz[1] as f32,
                    xyz[2] as f32,
                ))
                .with_parent_frame(format!("{entity_prefix}/urdf/{}", joint.parent.link))
                .with_child_frame(format!("{entity_prefix}/urdf/{}", joint.child.link)),
        )?;
    }

    re_log::info!(
        "Loaded URDF with {} links and {} joints",
        robot.links.len(),
        robot.joints.len()
    );

    Ok(())
}

fn bridge_supported_types() -> Vec<&'static str> {
    vec![
        "tf2_msgs::msg::TFMessage",
        "sensor_msgs::msg::Image",
        "sensor_msgs::msg::CompressedImage",
        "sensor_msgs::msg::PointCloud2",
        "sensor_msgs::msg::LaserScan",
        "sensor_msgs::msg::CameraInfo",
        "sensor_msgs::msg::Imu",
        "geometry_msgs::msg::PoseStamped",
        "nav_msgs::msg::Odometry",
        "nav_msgs::msg::Path",
        "nav_msgs::msg::OccupancyGrid",
        "visualization_msgs::msg::Marker",
        "visualization_msgs::msg::MarkerArray",
        "std_msgs::msg::String",
        "sensor_msgs::msg::Temperature",
        "sensor_msgs::msg::FluidPressure",
        "sensor_msgs::msg::RelativeHumidity",
        "sensor_msgs::msg::Illuminance",
    ]
}
