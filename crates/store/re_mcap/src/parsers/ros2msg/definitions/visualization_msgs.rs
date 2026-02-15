//! Definitions for the ROS2 `visualization_msgs` package.
//!
//! Based on definitions taken from <https://github.com/ros2/common_interfaces/tree/rolling/visualization_msgs>

use serde::{Deserialize, Serialize};

use super::builtin_interfaces::Time;
use super::geometry_msgs::{Point, Pose, Vector3};
use super::std_msgs::Header;

/// Represents a color in RGBA format with float components in the range [0.0, 1.0].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

/// Marker type constants.
#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
#[serde(from = "i32", into = "i32")]
#[repr(i32)]
pub enum MarkerType {
    Arrow = 0,
    Cube = 1,
    Sphere = 2,
    Cylinder = 3,
    LineStrip = 4,
    LineList = 5,
    CubeList = 6,
    SphereList = 7,
    Points = 8,
    TextViewFacing = 9,
    MeshResource = 10,
    TriangleList = 11,
}

impl From<i32> for MarkerType {
    fn from(value: i32) -> Self {
        match value {
            0 => Self::Arrow,
            1 => Self::Cube,
            2 => Self::Sphere,
            3 => Self::Cylinder,
            4 => Self::LineStrip,
            5 => Self::LineList,
            6 => Self::CubeList,
            7 => Self::SphereList,
            8 => Self::Points,
            9 => Self::TextViewFacing,
            10 => Self::MeshResource,
            11 => Self::TriangleList,
            _ => Self::Arrow, // default fallback
        }
    }
}

impl From<MarkerType> for i32 {
    fn from(value: MarkerType) -> Self {
        value as Self
    }
}

/// Marker action constants.
#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
#[serde(from = "i32", into = "i32")]
#[repr(i32)]
pub enum MarkerAction {
    /// Add or modify a marker (value 0). In ROS2, ADD and MODIFY are the same action.
    Add = 0,
    Delete = 2,
    DeleteAll = 3,
}

impl From<i32> for MarkerAction {
    fn from(value: i32) -> Self {
        match value {
            0 => Self::Add,
            2 => Self::Delete,
            3 => Self::DeleteAll,
            _ => Self::Add,
        }
    }
}

impl From<MarkerAction> for i32 {
    fn from(value: MarkerAction) -> Self {
        value as Self
    }
}

/// A visual marker for display in a 3D viewer.
#[derive(Debug, Serialize, Deserialize)]
pub struct Marker {
    pub header: Header,

    /// Namespace to place this object in, used with id to create a unique name.
    pub ns: String,

    /// Object ID useful in conjunction with the namespace for manipulating and deleting the object later.
    pub id: i32,

    /// Type of object.
    pub r#type: MarkerType,

    /// Action to take: ADD, DELETE, or DELETEALL.
    pub action: MarkerAction,

    /// Pose of the object.
    pub pose: Pose,

    /// Scale of the object 1,1,1 means default (usually 1m).
    pub scale: Vector3,

    /// Color of the object (RGBA, each in [0, 1]).
    pub color: ColorRGBA,

    /// How long the object should last before being automatically deleted.
    /// 0 means forever.
    pub lifetime: Time,

    /// If this marker should be frame-locked, i.e. retransformed into its frame every timestep.
    pub frame_locked: bool,

    /// Only used if the type specified has some use for them (e.g. POINTS, LINE_STRIP, etc.)
    pub points: Vec<Point>,

    /// Only used if the type specified has some use for them (e.g. POINTS, LINE_STRIP, etc.)
    /// The number of colors provided must either be 0 or equal to the number of points provided.
    pub colors: Vec<ColorRGBA>,

    /// Only used for text markers.
    pub text: String,

    /// Only used for MESH_RESOURCE markers.
    pub mesh_resource: String,

    /// If this marker should use the mesh/primitive's own colors.
    pub mesh_use_embedded_materials: bool,
}

/// An array of visual markers.
#[derive(Debug, Serialize, Deserialize)]
pub struct MarkerArray {
    pub markers: Vec<Marker>,
}
