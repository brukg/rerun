//! Zenoh session wrapper for managing connections.

use std::sync::Arc;

/// Wrapper around a Zenoh session providing convenient access and lifecycle management.
pub struct ZenohSession {
    session: Arc<zenoh::Session>,
}

impl ZenohSession {
    /// Connect to a Zenoh network using the provided configuration.
    pub async fn connect(config: zenoh::Config) -> anyhow::Result<Self> {
        re_log::info!("Connecting to Zenoh network\u{2026}");
        let session = zenoh::open(config)
            .await
            .map_err(|e| anyhow::anyhow!("Failed to open Zenoh session: {e}"))?;
        re_log::info!("Zenoh session established");
        Ok(Self {
            session: Arc::new(session),
        })
    }

    /// Connect to a Zenoh network with default configuration.
    pub async fn connect_default() -> anyhow::Result<Self> {
        Self::connect(zenoh::Config::default()).await
    }

    /// Get a reference to the underlying Zenoh session.
    pub fn session(&self) -> &Arc<zenoh::Session> {
        &self.session
    }

    /// Get a clone of the Arc-wrapped session for sharing across tasks.
    pub fn session_arc(&self) -> Arc<zenoh::Session> {
        Arc::clone(&self.session)
    }
}
