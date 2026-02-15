//! Main bridge orchestrator that routes Zenoh messages to the appropriate loggers.

use std::collections::HashMap;

use re_sdk::RecordingStream;
use re_zenoh_subscriber::ZenohMessage;

use crate::entity_mapping::EntityMapper;
use crate::logger_registry::{LoggerRegistry, TopicLogger};

/// Orchestrates the conversion of Zenoh ROS2 messages to Rerun archetypes.
pub struct ZenohBridge {
    rec: RecordingStream,
    entity_mapper: EntityMapper,
    logger_registry: LoggerRegistry,
    active_loggers: HashMap<String, Box<dyn TopicLogger>>,
}

impl ZenohBridge {
    /// Create a new bridge with the given recording stream and entity path prefix.
    pub fn new(rec: RecordingStream, entity_prefix: &str) -> Self {
        Self {
            rec,
            entity_mapper: EntityMapper::new(entity_prefix),
            logger_registry: LoggerRegistry::with_defaults(),
            active_loggers: HashMap::new(),
        }
    }

    /// Register a topic for processing. Creates the appropriate logger if the type is supported.
    /// Returns `true` if a logger was created.
    pub fn add_topic(&mut self, topic_name: &str, type_name: &str) -> bool {
        if self.active_loggers.contains_key(topic_name) {
            re_log::warn!("Topic {topic_name} already registered in bridge");
            return true;
        }

        if let Some(logger) = self.logger_registry.create_logger(type_name) {
            re_log::info!("Bridge: added logger for {topic_name} ({type_name})");
            self.active_loggers.insert(topic_name.to_owned(), logger);
            true
        } else {
            re_log::warn!("No logger available for message type {type_name} (topic: {topic_name})");
            false
        }
    }

    /// Remove a topic from the bridge.
    pub fn remove_topic(&mut self, topic_name: &str) {
        if self.active_loggers.remove(topic_name).is_some() {
            re_log::info!("Bridge: removed logger for {topic_name}");
        }
    }

    /// Process a single incoming Zenoh message.
    pub fn process_message(&mut self, msg: &ZenohMessage) -> anyhow::Result<()> {
        let entity_path = self.entity_mapper.map_topic(&msg.topic_name);

        if let Some(logger) = self.active_loggers.get_mut(&msg.topic_name) {
            let receive_time_ns =
                i64::try_from(msg.receive_time_ns).unwrap_or(i64::MAX);
            logger.log_message(&self.rec, &entity_path, &msg.payload, receive_time_ns)?;
        }

        Ok(())
    }

    /// Check if a message type is supported by this bridge.
    pub fn is_type_supported(&self, type_name: &str) -> bool {
        self.logger_registry.is_supported(type_name)
    }

    /// Get a reference to the recording stream.
    pub fn recording_stream(&self) -> &RecordingStream {
        &self.rec
    }

    /// Get a reference to the entity mapper.
    pub fn entity_mapper(&self) -> &EntityMapper {
        &self.entity_mapper
    }
}
