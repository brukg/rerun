//! Pose logger: `geometry_msgs/msg/PoseStamped` → `Transform3D` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::geometry_msgs::PoseStamped;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Transform3D;
use re_sdk_types::components::{RotationQuat, Translation3D};
use re_sdk_types::datatypes::Quaternion;

use crate::logger_registry::TopicLogger;

pub struct PoseLogger;

impl TopicLogger for PoseLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let pose_stamped = cdr::try_decode_message::<PoseStamped>(payload)?;

        let stamp_ns = pose_stamped.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        let pos = &pose_stamped.pose.position;
        let ori = &pose_stamped.pose.orientation;

        let translation = Translation3D::new(pos.x as f32, pos.y as f32, pos.z as f32);
        let quaternion: RotationQuat =
            Quaternion::from_xyzw([ori.x as f32, ori.y as f32, ori.z as f32, ori.w as f32]).into();

        rec.log(
            entity_path,
            &Transform3D::update_fields()
                .with_translation(translation)
                .with_quaternion(quaternion)
                .with_parent_frame(pose_stamped.header.frame_id),
        )?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "geometry_msgs::msg::PoseStamped"
    }
}
