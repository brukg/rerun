//! Native Zenoh topic visualization for ROS2, built on Rerun.
//!
//! This crate provides the `rerun-zenoh` binary that connects directly to a
//! Zenoh network (as used by `rmw_zenoh` in ROS2), discovers topics, and
//! visualizes them using the Rerun viewer.

pub mod app;
pub mod cli;
pub mod topic_selector;
