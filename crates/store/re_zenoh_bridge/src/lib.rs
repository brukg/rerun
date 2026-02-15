//! Deserializes CDR payloads from Zenoh and logs Rerun archetypes for ROS2 visualization.
//!
//! This crate is the core translation layer between ROS2 messages (received via Zenoh)
//! and Rerun's visualization primitives. It provides a [`TopicLogger`] trait for
//! per-message-type conversion and a [`ZenohBridge`] orchestrator.

mod bridge;
mod entity_mapping;
mod logger_registry;
pub mod loggers;

pub use bridge::ZenohBridge;
pub use entity_mapping::EntityMapper;
pub use logger_registry::{LoggerRegistry, TopicLogger};
