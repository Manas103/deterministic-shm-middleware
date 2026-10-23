// Subscribes to the frames ros2_crossing_pub publishes, in a separate OS
// process, and records one-way latency (this process's receipt timestamp
// minus the publisher's embedded send timestamp). Prints p50/p99 over
// every message received once the publisher's run ends.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

int main(int argc, char** argv) {
    int n_frames = 200;
    if (argc > 1) n_frames = atoi(argv[1]);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("shmmw_ros2_crossing_sub");

    std::vector<double> latencies_ms;
    latencies_ms.reserve(n_frames);

    auto sub = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/shmmw_perception/frame", 10,
        [&latencies_ms](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
            int64_t send_ts_ns;
            memcpy(&send_ts_ns, msg->data.data(), sizeof(send_ts_ns));
            int64_t recv_ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
            latencies_ms.push_back((recv_ts_ns - send_ts_ns) / 1.0e6);
        });

    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while ((int)latencies_ms.size() < n_frames && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(2));
    }
    rclcpp::shutdown();

    if (latencies_ms.empty()) {
        printf("received 0 messages\n");
        return 1;
    }
    // The first message pays one-time DDS discovery/handshake cost, not
    // representative of a running system's steady-state crossing cost;
    // excluded from the reported percentiles, included in the raw count.
    std::vector<double> steady(latencies_ms.begin() + (latencies_ms.size() > 1 ? 1 : 0), latencies_ms.end());
    std::sort(steady.begin(), steady.end());
    double p50 = steady[steady.size() / 2];
    double p99 = steady[(size_t)(0.99 * (steady.size() - 1))];
    printf("received=%zu first_ms=%.3f p50_ms=%.3f p99_ms=%.3f\n", latencies_ms.size(), latencies_ms.front(), p50, p99);
    return 0;
}
