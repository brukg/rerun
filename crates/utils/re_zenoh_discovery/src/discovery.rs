//! Topic discovery via Zenoh liveliness and admin space queries.

use std::collections::HashMap;
use std::sync::Arc;

use tokio::sync::RwLock;

use crate::ros2_keyexpr::{EndpointKind, Ros2KeyExpr, LIVELINESS_PREFIX};
use crate::session::ZenohSession;

/// Information about a discovered ROS2 topic.
#[derive(Debug, Clone)]
pub struct DiscoveredTopic {
    /// The ROS2 topic name (e.g., `/camera/color/image_raw`).
    pub topic_name: String,

    /// The ROS2 message type (e.g., `sensor_msgs::msg::Image`).
    pub type_name: String,

    /// The ROS2 domain ID.
    pub domain_id: u32,

    /// The Zenoh key-expression for subscribing to data on this topic.
    pub key_expr: String,

    /// Number of known publishers on this topic.
    pub publishers: usize,
}

/// Discovers ROS2 topics on the Zenoh network.
pub struct TopicDiscovery {
    session: Arc<ZenohSession>,
    topics: Arc<RwLock<HashMap<String, DiscoveredTopic>>>,
    domain_id: u32,
}

impl TopicDiscovery {
    /// Create a new topic discovery instance for the given Zenoh session.
    pub fn new(session: Arc<ZenohSession>, domain_id: u32) -> Self {
        Self {
            session,
            topics: Arc::new(RwLock::new(HashMap::new())),
            domain_id,
        }
    }

    /// Perform a one-shot scan of the Zenoh admin space for ROS2 topics.
    ///
    /// This queries the `@/<session_id>/**` admin space to find
    /// all active key-expressions that match the `rmw_zenoh` pattern.
    pub async fn scan(&self) -> anyhow::Result<()> {
        re_log::info!("Scanning for ROS2 topics on domain {}\u{2026}", self.domain_id);

        // Query the admin space for all publishers
        // The admin key pattern `@/**` discovers all resources
        let replies = self
            .session
            .session()
            .get("@/**")
            .await
            .map_err(|e| anyhow::anyhow!("Failed to query Zenoh admin space: {e}"))?;

        let mut topics = self.topics.write().await;
        let mut raw_count = 0u32;

        while let Ok(reply) = replies.recv_async().await {
            if let Ok(sample) = reply.into_result() {
                let key_str = sample.key_expr().as_str();
                raw_count += 1;
                re_log::debug!("Admin space key: {key_str}");

                if let Some(parsed) = Ros2KeyExpr::parse(key_str)
                    && parsed.domain_id == self.domain_id
                        && parsed.endpoint_kind == EndpointKind::Publisher
                    {
                        let entry =
                            topics
                                .entry(parsed.topic_name.clone())
                                .or_insert_with(|| DiscoveredTopic {
                                    topic_name: parsed.topic_name.clone(),
                                    type_name: parsed.type_name.clone(),
                                    domain_id: parsed.domain_id,
                                    key_expr: parsed.data_key_expr.clone(),
                                    publishers: 0,
                                });
                        entry.publishers += 1;
                    }
            }
        }

        re_log::info!(
            "Discovered {} ROS2 topics ({raw_count} raw admin entries)",
            topics.len()
        );
        Ok(())
    }

    /// Perform a scan using Zenoh liveliness tokens.
    ///
    /// This is the preferred discovery mechanism for `rmw_zenoh` which
    /// uses liveliness tokens to announce publishers and subscribers.
    /// Tokens are declared under the `@ros2_lv/` key prefix.
    pub async fn scan_liveliness(&self) -> anyhow::Result<()> {
        re_log::info!(
            "Scanning liveliness for ROS2 topics on domain {}\u{2026}",
            self.domain_id
        );

        // rmw_zenoh declares liveliness tokens under @ros2_lv/<domain_id>/...
        // The @ prefix is hermetic in zenoh — wildcards like ** never match it,
        // so we must query with the explicit prefix.
        let liveliness_pattern = format!("{LIVELINESS_PREFIX}/**");

        let replies = self
            .session
            .session()
            .liveliness()
            .get(&liveliness_pattern)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to query Zenoh liveliness: {e}"))?;

        let mut topics = self.topics.write().await;
        let mut raw_count = 0u32;

        while let Ok(reply) = replies.recv_async().await {
            if let Ok(sample) = reply.into_result() {
                let key_str = sample.key_expr().as_str();
                raw_count += 1;
                re_log::debug!("Liveliness token: {key_str}");

                if let Some(parsed) = Ros2KeyExpr::parse(key_str)
                    && parsed.domain_id == self.domain_id
                        && parsed.endpoint_kind == EndpointKind::Publisher
                    {
                        let entry =
                            topics
                                .entry(parsed.topic_name.clone())
                                .or_insert_with(|| DiscoveredTopic {
                                    topic_name: parsed.topic_name.clone(),
                                    type_name: parsed.type_name.clone(),
                                    domain_id: parsed.domain_id,
                                    key_expr: parsed.data_key_expr.clone(),
                                    publishers: 0,
                                });
                        entry.publishers += 1;
                    }
            }
        }

        re_log::info!(
            "Discovered {} ROS2 topics via liveliness ({raw_count} raw tokens)",
            topics.len()
        );
        Ok(())
    }

    /// Get a snapshot of all currently discovered topics.
    pub async fn topics(&self) -> Vec<DiscoveredTopic> {
        let topics = self.topics.read().await;
        let mut result: Vec<_> = topics.values().cloned().collect();
        result.sort_by(|a, b| a.topic_name.cmp(&b.topic_name));
        result
    }

    /// Get a specific discovered topic by name.
    pub async fn get_topic(&self, topic_name: &str) -> Option<DiscoveredTopic> {
        let topics = self.topics.read().await;
        topics.get(topic_name).cloned()
    }

    /// Clear all discovered topics.
    pub async fn clear(&self) {
        self.topics.write().await.clear();
    }

    /// Manually add a topic (useful for topics with known names that
    /// might not appear in discovery).
    pub async fn add_manual_topic(&self, topic_name: &str, type_name: &str) {
        let mut topics = self.topics.write().await;
        topics.insert(
            topic_name.to_owned(),
            DiscoveredTopic {
                topic_name: topic_name.to_owned(),
                type_name: type_name.to_owned(),
                domain_id: self.domain_id,
                key_expr: format!(
                    "{}/{}/**",
                    self.domain_id,
                    topic_name.trim_start_matches('/'),
                ),
                publishers: 0,
            },
        );
    }
}
