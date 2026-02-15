//! Interactive CLI topic selection.

use re_zenoh_discovery::DiscoveredTopic;

/// Display discovered topics and let the user select which to subscribe to.
///
/// Returns the indices of selected topics (0-based).
pub fn select_topics_interactive(
    topics: &[DiscoveredTopic],
    supported_types: &[&str],
) -> Vec<usize> {
    if topics.is_empty() {
        println!("No ROS2 topics discovered.");
        return Vec::new();
    }

    println!("\nDiscovered ROS2 topics:");
    println!("{:-<72}", "");

    for (i, topic) in topics.iter().enumerate() {
        let supported = if supported_types.contains(&topic.type_name.as_str()) {
            " "
        } else {
            "?"
        };
        println!(
            "  [{:>2}] {supported} {:<40} ({})",
            i + 1,
            topic.topic_name,
            topic.type_name
        );
    }

    println!("{:-<72}", "");
    println!("  ? = unsupported message type");
    println!();
    println!("Enter topic numbers (comma-separated), 'all' for all supported, or 'q' to quit:");

    let mut input = String::new();
    if std::io::stdin().read_line(&mut input).is_err() {
        return Vec::new();
    }
    let input = input.trim();

    if input.eq_ignore_ascii_case("q") || input.eq_ignore_ascii_case("quit") {
        return Vec::new();
    }

    if input.eq_ignore_ascii_case("all") {
        return topics
            .iter()
            .enumerate()
            .filter(|(_, t)| supported_types.contains(&t.type_name.as_str()))
            .map(|(i, _)| i)
            .collect();
    }

    input
        .split(',')
        .filter_map(|s| {
            let s = s.trim();
            let n = s.parse::<usize>().ok()?;
            if n >= 1 && n <= topics.len() {
                                Some(n - 1)
                            } else {
                                eprintln!("Invalid topic number: {n}");
                                None
                            }
        })
        .collect()
}

/// Automatically select all topics with supported message types.
pub fn select_all_supported(
    topics: &[DiscoveredTopic],
    supported_types: &[&str],
) -> Vec<usize> {
    topics
        .iter()
        .enumerate()
        .filter(|(_, t)| supported_types.contains(&t.type_name.as_str()))
        .map(|(i, _)| i)
        .collect()
}
