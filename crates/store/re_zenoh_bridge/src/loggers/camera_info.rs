//! `CameraInfo` logger: `sensor_msgs/msg/CameraInfo` → `Pinhole` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs::CameraInfo;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Pinhole;

use crate::logger_registry::TopicLogger;

pub struct CameraInfoLogger;

impl TopicLogger for CameraInfoLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let cam = cdr::try_decode_message::<CameraInfo>(payload)?;

        let stamp_ns = cam.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        // Extract focal lengths and principal point from the K matrix (3x3 row-major).
        // K = [fx, 0, cx,
        //      0, fy, cy,
        //      0,  0,  1]
        let fx = cam.k[0] as f32;
        let fy = cam.k[4] as f32;
        let cx = cam.k[2] as f32;
        let cy = cam.k[5] as f32;

        let pinhole = Pinhole::from_focal_length_and_resolution(
            [fx, fy],
            [cam.width as f32, cam.height as f32],
        )
        .with_principal_point([cx, cy]);

        // Log the pinhole camera model on the image plane entity
        // (append /image_plane to distinguish from the 3D camera entity)
        let image_plane_path = format!("{entity_path}/image_plane");
        rec.log(image_plane_path, &pinhole)?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::CameraInfo"
    }
}
