//! `LaserScan` logger: `sensor_msgs/msg/LaserScan` → `Points3D` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs::LaserScan;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Points3D;

use crate::logger_registry::TopicLogger;

pub struct LaserScanLogger;

impl TopicLogger for LaserScanLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let scan = cdr::try_decode_message::<LaserScan>(payload)?;

        let stamp_ns = scan.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        // Convert polar coordinates to 3D Cartesian points (in the scanner's frame, z=0)
        let mut positions = Vec::with_capacity(scan.ranges.len());
        for (i, &range) in scan.ranges.iter().enumerate() {
            // Skip invalid ranges
            if range < scan.range_min || range > scan.range_max || !range.is_finite() {
                continue;
            }

            let angle = scan.angle_min + (i as f32) * scan.angle_increment;
            let x = range * angle.cos();
            let y = range * angle.sin();
            positions.push([x, y, 0.0_f32]);
        }

        let points = Points3D::new(positions);

        rec.log(entity_path, &points)?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::LaserScan"
    }
}
