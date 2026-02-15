//! Marker logger: `visualization_msgs/msg/Marker[Array]` → various Rerun primitives.

use re_mcap::cdr;
use re_mcap::ros2_definitions::visualization_msgs::{Marker, MarkerAction, MarkerArray, MarkerType};
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::{
    Arrows3D, Boxes3D, Ellipsoids3D, LineStrips3D, Points3D, TextLog,
};
use re_sdk_types::components::Color;

use crate::logger_registry::TopicLogger;

pub struct MarkerLogger;

impl TopicLogger for MarkerLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let marker = cdr::try_decode_message::<Marker>(payload)?;

        let stamp_ns = marker.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        log_single_marker(rec, entity_path, &marker)?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "visualization_msgs::msg::Marker"
    }
}

pub struct MarkerArrayLogger;

impl TopicLogger for MarkerArrayLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let marker_array = cdr::try_decode_message::<MarkerArray>(payload)?;

        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        for marker in &marker_array.markers {
            let stamp_ns = marker.header.stamp.as_nanos();
            rec.set_time_sequence("ros2_time", stamp_ns);

            let marker_path = format!("{entity_path}/{}/{}", marker.ns, marker.id);
            log_single_marker(rec, &marker_path, marker)?;
        }

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "visualization_msgs::msg::MarkerArray"
    }
}

fn marker_color(marker: &Marker) -> Color {
    Color::from_unmultiplied_rgba(
        (marker.color.r * 255.0) as u8,
        (marker.color.g * 255.0) as u8,
        (marker.color.b * 255.0) as u8,
        (marker.color.a * 255.0) as u8,
    )
}

fn log_single_marker(
    rec: &RecordingStream,
    entity_path: &str,
    marker: &Marker,
) -> anyhow::Result<()> {
    // Handle DELETE and DELETEALL actions
    match marker.action {
        MarkerAction::Delete | MarkerAction::DeleteAll => {
            // Clear the entity by logging an empty archetype
            // For now we just skip; full clearing requires Rerun's clear API
            return Ok(());
        }
        MarkerAction::Add => {}
    }

    let color = marker_color(marker);
    let sx = marker.scale.x as f32;
    let sy = marker.scale.y as f32;
    let sz = marker.scale.z as f32;

    match marker.r#type {
        MarkerType::Arrow => {
            if marker.points.len() >= 2 {
                let p0 = &marker.points[0];
                let p1 = &marker.points[1];
                let vector = [
                    (p1.x - p0.x) as f32,
                    (p1.y - p0.y) as f32,
                    (p1.z - p0.z) as f32,
                ];
                let origin = [p0.x as f32, p0.y as f32, p0.z as f32];
                rec.log(
                    entity_path,
                    &Arrows3D::from_vectors([vector])
                        .with_origins([origin])
                        .with_colors([color]),
                )?;
            } else {
                // Arrow with scale: shaft length = sx, shaft diameter = sy, head diameter = sz
                rec.log(
                    entity_path,
                    &Arrows3D::from_vectors([[sx, 0.0, 0.0]]).with_colors([color]),
                )?;
            }
        }

        MarkerType::Cube | MarkerType::CubeList => {
            if marker.r#type as i32 == MarkerType::CubeList as i32 && !marker.points.is_empty() {
                let centers: Vec<[f32; 3]> = marker
                    .points
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                    .collect();
                let half_sizes = vec![[sx / 2.0, sy / 2.0, sz / 2.0]; centers.len()];
                rec.log(
                    entity_path,
                    &Boxes3D::from_centers_and_half_sizes(centers, half_sizes).with_colors([color]),
                )?;
            } else {
                rec.log(
                    entity_path,
                    &Boxes3D::from_half_sizes([[sx / 2.0, sy / 2.0, sz / 2.0]])
                        .with_colors([color]),
                )?;
            }
        }

        MarkerType::Sphere | MarkerType::SphereList => {
            if marker.r#type as i32 == MarkerType::SphereList as i32 && !marker.points.is_empty() {
                let centers: Vec<[f32; 3]> = marker
                    .points
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                    .collect();
                let half_sizes = vec![[sx / 2.0, sy / 2.0, sz / 2.0]; centers.len()];
                rec.log(
                    entity_path,
                    &Ellipsoids3D::from_centers_and_half_sizes(centers, half_sizes)
                        .with_colors([color]),
                )?;
            } else {
                rec.log(
                    entity_path,
                    &Ellipsoids3D::from_half_sizes([[sx / 2.0, sy / 2.0, sz / 2.0]])
                        .with_colors([color]),
                )?;
            }
        }

        MarkerType::Cylinder => {
            rec.log(
                entity_path,
                &Ellipsoids3D::from_half_sizes([[sx / 2.0, sy / 2.0, sz / 2.0]])
                    .with_colors([color]),
            )?;
        }

        MarkerType::LineStrip => {
            if !marker.points.is_empty() {
                let points: Vec<[f32; 3]> = marker
                    .points
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                    .collect();
                rec.log(
                    entity_path,
                    &LineStrips3D::new([points]).with_colors([color]),
                )?;
            }
        }

        MarkerType::LineList => {
            if marker.points.len() >= 2 {
                let segments: Vec<Vec<[f32; 3]>> = marker
                    .points
                    .chunks(2)
                    .filter(|c| c.len() == 2)
                    .map(|c| {
                        vec![
                            [c[0].x as f32, c[0].y as f32, c[0].z as f32],
                            [c[1].x as f32, c[1].y as f32, c[1].z as f32],
                        ]
                    })
                    .collect();
                rec.log(
                    entity_path,
                    &LineStrips3D::new(segments).with_colors([color]),
                )?;
            }
        }

        MarkerType::Points => {
            if !marker.points.is_empty() {
                let positions: Vec<[f32; 3]> = marker
                    .points
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                    .collect();

                let mut points = Points3D::new(positions);

                if marker.colors.len() == marker.points.len() {
                    let colors: Vec<Color> = marker
                        .colors
                        .iter()
                        .map(|c| {
                            Color::from_unmultiplied_rgba(
                                (c.r * 255.0) as u8,
                                (c.g * 255.0) as u8,
                                (c.b * 255.0) as u8,
                                (c.a * 255.0) as u8,
                            )
                        })
                        .collect();
                    points = points.with_colors(colors);
                } else {
                    points = points.with_colors([color]);
                }

                rec.log(entity_path, &points)?;
            }
        }

        MarkerType::TextViewFacing => {
            rec.log(entity_path, &TextLog::new(marker.text.clone()))?;
        }

        MarkerType::TriangleList => {
            // Triangle lists require mesh logging which is more complex.
            // For now, log the vertices as points.
            if !marker.points.is_empty() {
                let positions: Vec<[f32; 3]> = marker
                    .points
                    .iter()
                    .map(|p| [p.x as f32, p.y as f32, p.z as f32])
                    .collect();
                rec.log(
                    entity_path,
                    &Points3D::new(positions).with_colors([color]),
                )?;
            }
        }

        MarkerType::MeshResource => {
            // Mesh resources require loading external files.
            // Log a placeholder text for now.
            re_log::debug_once!(
                "MeshResource markers not yet supported: {}",
                marker.mesh_resource
            );
        }
    }

    Ok(())
}
