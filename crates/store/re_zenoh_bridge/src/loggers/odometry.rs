//! Odometry logger: `nav_msgs/msg/Odometry` → `Transform3D` + `Arrows3D` archetypes.

use re_mcap::cdr;
use re_mcap::ros2_definitions::nav_msgs::Odometry;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::{Arrows3D, Transform3D};
use re_sdk_types::components::{RotationQuat, Translation3D};
use re_sdk_types::datatypes::Quaternion;

use crate::logger_registry::TopicLogger;

pub struct OdometryLogger;

impl TopicLogger for OdometryLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let odom = cdr::try_decode_message::<Odometry>(payload)?;

        let stamp_ns = odom.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        let pos = &odom.pose.pose.position;
        let ori = &odom.pose.pose.orientation;

        let translation = Translation3D::new(pos.x as f32, pos.y as f32, pos.z as f32);
        let quaternion: RotationQuat =
            Quaternion::from_xyzw([ori.x as f32, ori.y as f32, ori.z as f32, ori.w as f32]).into();

        // Log the pose as a transform
        rec.log(
            entity_path,
            &Transform3D::update_fields()
                .with_translation(translation)
                .with_quaternion(quaternion)
                .with_parent_frame(odom.header.frame_id.clone())
                .with_child_frame(odom.child_frame_id.clone()),
        )?;

        // Log velocity as an arrow at the pose location
        let vel = &odom.twist.twist;
        let speed = (vel.linear.x * vel.linear.x
            + vel.linear.y * vel.linear.y
            + vel.linear.z * vel.linear.z)
            .sqrt();

        if speed > 1e-6 {
            let velocity_path = format!("{entity_path}/velocity");
            rec.log(
                velocity_path,
                &Arrows3D::from_vectors([[
                    vel.linear.x as f32,
                    vel.linear.y as f32,
                    vel.linear.z as f32,
                ]]),
            )?;
        }

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "nav_msgs::msg::Odometry"
    }
}
