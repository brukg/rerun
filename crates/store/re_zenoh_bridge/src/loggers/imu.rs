//! IMU logger: `sensor_msgs/msg/Imu` → `Scalar` archetypes (6-axis time series).

use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs::Imu;
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Scalars;

use crate::logger_registry::TopicLogger;

pub struct ImuLogger;

impl TopicLogger for ImuLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let imu = cdr::try_decode_message::<Imu>(payload)?;

        let stamp_ns = imu.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        // Log linear acceleration as 3 scalar time series
        let accel = &imu.linear_acceleration;
        rec.log(
            format!("{entity_path}/linear_acceleration/x"),
            &Scalars::single(accel.x),
        )?;
        rec.log(
            format!("{entity_path}/linear_acceleration/y"),
            &Scalars::single(accel.y),
        )?;
        rec.log(
            format!("{entity_path}/linear_acceleration/z"),
            &Scalars::single(accel.z),
        )?;

        // Log angular velocity as 3 scalar time series
        let gyro = &imu.angular_velocity;
        rec.log(
            format!("{entity_path}/angular_velocity/x"),
            &Scalars::single(gyro.x),
        )?;
        rec.log(
            format!("{entity_path}/angular_velocity/y"),
            &Scalars::single(gyro.y),
        )?;
        rec.log(
            format!("{entity_path}/angular_velocity/z"),
            &Scalars::single(gyro.z),
        )?;

        // Log orientation as a transform if the covariance is not all -1
        // (first element of -1 means orientation data is not available)
        if imu.orientation_covariance[0] >= 0.0 {
            let ori = &imu.orientation;
            rec.log(
                format!("{entity_path}/orientation"),
                &re_sdk_types::archetypes::Transform3D::update_fields().with_quaternion(
                    re_sdk_types::components::RotationQuat::from(
                        re_sdk_types::datatypes::Quaternion::from_xyzw([
                            ori.x as f32,
                            ori.y as f32,
                            ori.z as f32,
                            ori.w as f32,
                        ]),
                    ),
                ),
            )?;
        }

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::Imu"
    }
}
