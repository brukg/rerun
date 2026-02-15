//! Compressed image logger: `sensor_msgs/msg/CompressedImage` → `EncodedImage` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::EncodedImage;

use crate::logger_registry::TopicLogger;

pub struct CompressedImageLogger;

impl TopicLogger for CompressedImageLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let img = cdr::try_decode_message::<sensor_msgs::CompressedImage<'_>>(payload)?;

        let stamp_ns = img.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        // Determine media type from the format field
        let media_type = if img.format.contains("jpeg") || img.format.contains("jpg") {
            Some("image/jpeg")
        } else if img.format.contains("png") {
            Some("image/png")
        } else {
            None
        };

        let mut encoded = EncodedImage::from_file_contents(img.data.into_owned());
        if let Some(mt) = media_type {
            encoded = encoded.with_media_type(mt);
        }

        rec.log(entity_path, &encoded)?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::CompressedImage"
    }
}
