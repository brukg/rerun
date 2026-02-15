//! `OccupancyGrid` logger: `nav_msgs/msg/OccupancyGrid` → `Image` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::nav_msgs::OccupancyGrid;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::{Image, Transform3D};
use re_sdk_types::components::{RotationQuat, Translation3D};
use re_sdk_types::datatypes::{ChannelDatatype, ColorModel, ImageFormat, Quaternion};

use crate::logger_registry::TopicLogger;

pub struct OccupancyGridLogger;

impl TopicLogger for OccupancyGridLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let grid = cdr::try_decode_message::<OccupancyGrid>(payload)?;

        let stamp_ns = grid.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        let width = grid.info.width;
        let height = grid.info.height;

        if width == 0 || height == 0 {
            return Ok(());
        }

        let total_cells = (width as u64) * (height as u64);
        if total_cells > 16_000_000 {
            anyhow::bail!(
                "OccupancyGrid {width}x{height} ({total_cells} cells) exceeds 16M cell limit"
            );
        }

        // Convert occupancy values to grayscale pixels:
        // -1 (unknown) → gray (128)
        // 0 (free) → white (254)
        // 100 (occupied) → black (0)
        let pixels: Vec<u8> = grid
            .data
            .iter()
            .map(|&val| {
                if val < 0 {
                    128u8 // unknown
                } else {
                    let clamped = val.clamp(0, 100) as u8;
                    254 - (clamped as u16 * 254 / 100) as u8
                }
            })
            .collect();

        let format =
            ImageFormat::from_color_model([width, height], ColorModel::L, ChannelDatatype::U8);
        rec.log(entity_path, &Image::new(pixels, format))?;

        // Log the grid's spatial position as a transform
        let origin = &grid.info.origin;
        let pos = &origin.position;
        let ori = &origin.orientation;

        let transform_path = format!("{entity_path}/transform");
        rec.log(
            transform_path,
            &Transform3D::update_fields()
                .with_translation(Translation3D::new(
                    pos.x as f32,
                    pos.y as f32,
                    pos.z as f32,
                ))
                .with_quaternion(RotationQuat::from(Quaternion::from_xyzw([
                    ori.x as f32,
                    ori.y as f32,
                    ori.z as f32,
                    ori.w as f32,
                ])))
                .with_parent_frame(grid.header.frame_id),
        )?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "nav_msgs::msg::OccupancyGrid"
    }
}
