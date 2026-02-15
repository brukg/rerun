//! Entry point for the rerun-zenoh binary.

use clap::Parser as _;

use rerun_zenoh::cli::Cli;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // Initialize logging
    re_log::setup_logging();

    let cli = Cli::parse();

    rerun_zenoh::app::run(cli).await
}
