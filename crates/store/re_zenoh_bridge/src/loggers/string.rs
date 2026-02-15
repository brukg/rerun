//! String logger: `std_msgs/msg/String` → `TextLog` archetype.

use re_mcap::cdr;
use re_mcap::ros2_definitions::std_msgs::StringMessage;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::TextLog;

use crate::logger_registry::TopicLogger;

pub struct StringLogger;

impl TopicLogger for StringLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let msg = cdr::try_decode_message::<StringMessage>(payload)?;

        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        rec.log(entity_path, &TextLog::new(msg.data))?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "std_msgs::msg::String"
    }
}
