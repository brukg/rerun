//! `PointCloud2` logger: `sensor_msgs/msg/PointCloud2` → `Points3D` archetype.

use std::io::Cursor;

use byteorder::{BigEndian, LittleEndian, ReadBytesExt as _};
use re_mcap::cdr;
use re_mcap::ros2_definitions::sensor_msgs::{PointCloud2, PointField, PointFieldDatatype};
use re_sdk::RecordingStream;
use re_sdk_types::archetypes::Points3D;
use re_sdk_types::components::Color;

use crate::logger_registry::TopicLogger;

const MAX_POINTS: usize = 5_000_000;

#[derive(Default)]
pub struct PointCloudLogger {
    positions: Vec<[f32; 3]>,
    colors: Vec<Color>,
}

impl PointCloudLogger {
    pub fn new() -> Self {
        Self::default()
    }
}

impl TopicLogger for PointCloudLogger {
    fn log_message(
        &mut self,
        rec: &RecordingStream,
        entity_path: &str,
        payload: &[u8],
        receive_time_ns: i64,
    ) -> anyhow::Result<()> {
        let pc = cdr::try_decode_message::<PointCloud2>(payload)?;

        let stamp_ns = pc.header.stamp.as_nanos();
        rec.set_time_sequence("ros2_time", stamp_ns);
        rec.set_timestamp_nanos_since_epoch("receive_time", receive_time_ns);

        // Find x, y, z field offsets
        let x_field = find_field(&pc.fields, "x");
        let y_field = find_field(&pc.fields, "y");
        let z_field = find_field(&pc.fields, "z");

        let (Some(x_field), Some(y_field), Some(z_field)) = (x_field, y_field, z_field) else {
            re_log::warn_once!("PointCloud2 missing x/y/z fields, skipping");
            return Ok(());
        };

        let step = pc.point_step as usize;
        if step == 0 {
            return Ok(());
        }
        let num_points = (pc.data.len() / step)
            .min((pc.height * pc.width) as usize)
            .min(MAX_POINTS);
        let is_be = pc.is_bigendian;

        // Reuse position buffer across messages (clear resets length, keeps allocation)
        self.positions.clear();
        for i in 0..num_points {
            let offset = i * step;
            let point_data = &pc.data[offset..offset + step];

            let x = read_f32(point_data, x_field.offset as usize, x_field.datatype, is_be);
            let y = read_f32(point_data, y_field.offset as usize, y_field.datatype, is_be);
            let z = read_f32(point_data, z_field.offset as usize, z_field.datatype, is_be);

            if x.is_finite() && y.is_finite() && z.is_finite() {
                self.positions.push([x, y, z]);
            }
        }

        // Try to find RGB color field
        let rgb_field = find_field(&pc.fields, "rgb")
            .or_else(|| find_field(&pc.fields, "rgba"));

        let mut points = Points3D::new(&self.positions[..]);

        if let Some(rgb_f) = rgb_field {
            let color_end = rgb_f.offset as usize + 4;
            if color_end <= step {
                // Reuse color buffer
                self.colors.clear();
                for i in 0..num_points {
                    let offset = i * step + rgb_f.offset as usize;
                    if offset + 4 <= pc.data.len() {
                        let rgba_bytes = &pc.data[offset..offset + 4];
                        // ROS2 typically packs RGB as float32 with bytes [B, G, R, A] in little-endian
                        let r = rgba_bytes[2];
                        let g = rgba_bytes[1];
                        let b = rgba_bytes[0];
                        let a = if rgba_bytes[3] > 0 { rgba_bytes[3] } else { 255 };
                        self.colors.push(Color::from_unmultiplied_rgba(r, g, b, a));
                    }
                }
                if !self.colors.is_empty() {
                    points = points.with_colors(self.colors.iter().copied());
                }
            }
        }

        rec.log(entity_path, &points)?;

        Ok(())
    }

    fn type_name(&self) -> &'static str {
        "sensor_msgs::msg::PointCloud2"
    }
}

fn find_field<'a>(fields: &'a [PointField], name: &str) -> Option<&'a PointField> {
    fields.iter().find(|f| f.name == name)
}

fn read_f32(data: &[u8], offset: usize, datatype: PointFieldDatatype, is_big_endian: bool) -> f32 {
    let byte_size = datatype_size(datatype);
    if offset + byte_size > data.len() {
        return 0.0;
    }
    let data = &data[offset..];
    let mut cursor = Cursor::new(data);
    match (is_big_endian, datatype) {
        (_, PointFieldDatatype::UInt8) => cursor.read_u8().unwrap_or(0) as f32,
        (_, PointFieldDatatype::Int8) => cursor.read_i8().unwrap_or(0) as f32,
        (true, PointFieldDatatype::Int16) => cursor.read_i16::<BigEndian>().unwrap_or(0) as f32,
        (true, PointFieldDatatype::UInt16) => cursor.read_u16::<BigEndian>().unwrap_or(0) as f32,
        (true, PointFieldDatatype::Int32) => cursor.read_i32::<BigEndian>().unwrap_or(0) as f32,
        (true, PointFieldDatatype::UInt32) => cursor.read_u32::<BigEndian>().unwrap_or(0) as f32,
        (true, PointFieldDatatype::Float32) => cursor.read_f32::<BigEndian>().unwrap_or(0.0),
        (true, PointFieldDatatype::Float64) => cursor.read_f64::<BigEndian>().unwrap_or(0.0) as f32,
        (false, PointFieldDatatype::Int16) => cursor.read_i16::<LittleEndian>().unwrap_or(0) as f32,
        (false, PointFieldDatatype::UInt16) => cursor.read_u16::<LittleEndian>().unwrap_or(0) as f32,
        (false, PointFieldDatatype::Int32) => cursor.read_i32::<LittleEndian>().unwrap_or(0) as f32,
        (false, PointFieldDatatype::UInt32) => cursor.read_u32::<LittleEndian>().unwrap_or(0) as f32,
        (false, PointFieldDatatype::Float32) => cursor.read_f32::<LittleEndian>().unwrap_or(0.0),
        (false, PointFieldDatatype::Float64) => cursor.read_f64::<LittleEndian>().unwrap_or(0.0) as f32,
    }
}

fn datatype_size(dt: PointFieldDatatype) -> usize {
    match dt {
        PointFieldDatatype::Int8 | PointFieldDatatype::UInt8 => 1,
        PointFieldDatatype::Int16 | PointFieldDatatype::UInt16 => 2,
        PointFieldDatatype::Int32 | PointFieldDatatype::UInt32 | PointFieldDatatype::Float32 => 4,
        PointFieldDatatype::Float64 => 8,
    }
}
