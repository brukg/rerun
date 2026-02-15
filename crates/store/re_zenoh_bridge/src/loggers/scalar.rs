//! Scalar logger for numeric sensor messages → `Scalar` archetype.
//!
//! Handles `sensor_msgs/msg/Temperature`, `FluidPressure`, `RelativeHumidity`, `Illuminance`.

use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs::{FluidPressure, Illuminance, RelativeHumidity, Temperature};
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Scalars;

use crate::logger_registry::TopicLogger;

pub struct ScalarLogger {
    type_name: String,
}

impl ScalarLogger {
    pub fn new(type_name: String) -> Self {
        Self { type_name }
    }
}

impl TopicLogger for ScalarLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        match self.type_name.as_str() {
            "sensor_msgs::msg::Temperature" => {
                let msg = cdr::try_decode_message::<Temperature>(payload)?;
                rec.set_time_sequence("ros2_time", msg.header.stamp.as_nanos());
                rec.log(entity_path, &Scalars::single(msg.temperature))?;
            }
            "sensor_msgs::msg::FluidPressure" => {
                let msg = cdr::try_decode_message::<FluidPressure>(payload)?;
                rec.set_time_sequence("ros2_time", msg.header.stamp.as_nanos());
                rec.log(entity_path, &Scalars::single(msg.fluid_pressure))?;
            }
            "sensor_msgs::msg::RelativeHumidity" => {
                let msg = cdr::try_decode_message::<RelativeHumidity>(payload)?;
                rec.set_time_sequence("ros2_time", msg.header.stamp.as_nanos());
                rec.log(entity_path, &Scalars::single(msg.humidity))?;
            }
            "sensor_msgs::msg::Illuminance" => {
                let msg = cdr::try_decode_message::<Illuminance>(payload)?;
                rec.set_time_sequence("ros2_time", msg.header.stamp.as_nanos());
                rec.log(entity_path, &Scalars::single(msg.illuminance))?;
            }
            _ => {
                re_log::warn_once!("Unknown scalar type: {}", self.type_name);
            }
        }

        Ok(())
    }

    fn type_name(&self) -> &str {
        &self.type_name
    }
}
