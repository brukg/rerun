//! TF logger: `tf2_msgs/msg/TFMessage` → `Transform3D` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::tf2_msgs::TFMessage;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Transform3D;
use re_sdk_types::components::{RotationQuat, Translation3D};
use re_sdk_types::datatypes::Quaternion;

use crate::logger_registry::TopicLogger;

pub struct TfLogger;

impl TopicLogger for TfLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        _entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let tf_msg = cdr::try_decode_message::<TFMessage>(payload)?;

        for transform_stamped in &tf_msg.transforms {
            let stamp_ns = transform_stamped.header.stamp.as_nanos();
            rec.set_time_sequence("ros2_time", stamp_ns);
            rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

            let t = &transform_stamped.transform;
            let translation = Translation3D::new(
                t.translation.x as f32,
                t.translation.y as f32,
                t.translation.z as f32,
            );
            let quaternion: RotationQuat = Quaternion::from_xyzw([
                t.rotation.x as f32,
                t.rotation.y as f32,
                t.rotation.z as f32,
                t.rotation.w as f32,
            ])
            .into();

            let parent = &transform_stamped.header.frame_id;
            let child = &transform_stamped.child_frame_id;

            // Log the transform using the child frame as the entity path
            let entity_path = format!("ros2/tf/{child}");
            rec.log(
                entity_path,
                &Transform3D::update_fields()
                    .with_translation(translation)
                    .with_quaternion(quaternion)
                    .with_parent_frame(parent.clone())
                    .with_child_frame(child.clone()),
            )?;
        }

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "tf2_msgs::msg::TFMessage"
    }
}
