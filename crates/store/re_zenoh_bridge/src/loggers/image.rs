//! Image logger: `sensor_msgs/msg/Image` → `Image` / `DepthImage` archetype.

use re_mcap::cdr;
use re_mcap::decode_image_format;
use re_mcap::ros2_definitions::sensor_msgs;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::{DepthImage, Image};

use crate::logger_registry::TopicLogger;

pub struct ImageLogger;

impl TopicLogger for ImageLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let img = cdr::try_decode_message::<sensor_msgs::Image<'_>>(payload)?;

        let stamp_ns = img.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        let dimensions = [img.width, img.height];
        let img_format = decode_image_format(&img.encoding, dimensions)?;

        // `color_model` is `None` for depth formats
        if img_format.color_model.is_none() {
            rec.log(
                entity_path,
                &DepthImage::new(img.data.into_owned(), img_format),
            )?;
        } else {
            rec.log(
                entity_path,
                &Image::new(img.data.into_owned(), img_format),
            )?;
        }

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::Image"
    }
}
