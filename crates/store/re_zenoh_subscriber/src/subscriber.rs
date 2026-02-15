//! Per-topic Zenoh subscriber management.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use tokio::sync::mpsc;
use zenoh::Wait as _;

use re_zenoh_discovery::{DiscoveredTopic, ZenohSession};

use crate::message::ZenohMessage;

/// Manages subscriptions to individual Zenoh topics.
///
/// Each subscribed topic gets its own Zenoh subscriber. Received messages
/// are forwarded through a bounded mpsc channel as [`ZenohMessage`]s.
pub struct SubscriptionManager {
    session: Arc<ZenohSession>,
    subscribers: HashMap<String, zenoh::pubsub::Subscriber<()>>,
    message_tx: mpsc::Sender<ZenohMessage>,
    message_rx: Option<mpsc::Receiver<ZenohMessage>>,
}

/// Channel capacity for incoming Zenoh messages (applies backpressure when full).
const MESSAGE_CHANNEL_CAPACITY: usize = 4096;

impl SubscriptionManager {
    /// Create a new subscription manager.
    pub fn new(session: Arc<ZenohSession>) -> Self {
        let (tx, rx) = mpsc::channel(MESSAGE_CHANNEL_CAPACITY);
        Self {
            session,
            subscribers: HashMap::new(),
            message_tx: tx,
            message_rx: Some(rx),
        }
    }

    /// Take the message receiver. Can only be called once.
    pub fn take_message_receiver(&mut self) -> Option<mpsc::Receiver<ZenohMessage>> {
        self.message_rx.take()
    }

    /// Subscribe to a discovered topic.
    pub fn subscribe(&mut self, topic: &DiscoveredTopic) -> anyhow::Result<()> {
        if self.subscribers.contains_key(&topic.topic_name) {
            re_log::warn!("Already subscribed to {}", topic.topic_name);
            return Ok(());
        }

        let topic_name = topic.topic_name.clone();
        let type_name = topic.type_name.clone();
        let tx = self.message_tx.clone();

        // Use the data key expression from discovery.
        // For rmw_zenoh topics this is: <domain_id>/<topic>/<dds_type>/<type_hash>
        let key_expr = topic.key_expr.clone();

        re_log::info!("Subscribing to {} ({})", topic_name, type_name);

        let subscriber = self
            .session
            .session()
            .declare_subscriber(&key_expr)
            .callback(move |sample| {
                let now = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;

                let payload = sample.payload().to_bytes().to_vec();

                let msg = ZenohMessage {
                    topic_name: topic_name.clone(),
                    type_name: type_name.clone(),
                    payload,
                    receive_time_ns: now,
                };

                if tx.try_send(msg).is_err() {
                    re_log::debug!("Message channel full or receiver dropped");
                }
            })
            .wait()
            .map_err(|e| anyhow::anyhow!("Failed to create subscriber for {key_expr}: {e}"))?;

        self.subscribers
            .insert(topic.topic_name.clone(), subscriber);
        re_log::info!("Subscribed to {}", topic.topic_name);

        Ok(())
    }

    /// Unsubscribe from a topic.
    pub fn unsubscribe(&mut self, topic_name: &str) {
        if let Some(_subscriber) = self.subscribers.remove(topic_name) {
            re_log::info!("Unsubscribed from {topic_name}");
            // Subscriber is dropped, which closes the subscription
        } else {
            re_log::warn!("Not subscribed to {topic_name}");
        }
    }

    /// Check if a topic is currently subscribed.
    pub fn is_subscribed(&self, topic_name: &str) -> bool {
        self.subscribers.contains_key(topic_name)
    }

    /// Get the list of currently subscribed topic names.
    pub fn subscribed_topics(&self) -> Vec<String> {
        self.subscribers.keys().cloned().collect()
    }

    /// Unsubscribe from all topics.
    pub fn unsubscribe_all(&mut self) {
        let topics: Vec<String> = self.subscribers.keys().cloned().collect();
        for topic in topics {
            self.unsubscribe(&topic);
        }
    }
}
