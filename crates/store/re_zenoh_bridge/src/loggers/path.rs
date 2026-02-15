//! Path logger: `nav_msgs/msg/Path` → `LineStrips3D` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::nav_msgs::Path;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::LineStrips3D;

use crate::logger_registry::TopicLogger;

pub struct PathLogger;

impl TopicLogger for PathLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let path = cdr::try_decode_message::<Path>(payload)?;

        let stamp_ns = path.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        if path.poses.is_empty() {
            return Ok(());
        }

        let points: Vec<[f32; 3]> = path
            .poses
            .iter()
            .map(|ps| {
                let p = &ps.pose.position;
                [p.x as f32, p.y as f32, p.z as f32]
            })
            .collect();

        rec.log(entity_path, &LineStrips3D::new([points]))?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "nav_msgs::msg::Path"
    }
}
