// Publishes M synthetic-frame-sized messages over ROS 2 (rclcpp + a plain
// std_msgs::msg::UInt8MultiArray; no custom .msg/rosidl generation needed)
// and measures the cost of actually crossing to a subscriber living in
// another process (ros2_crossing_sub), which is the question this file
// exists to answer: how much more does a ROS 2 process boundary cost than
// the shared-memory hop the rest of this repo measures elsewhere.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "shmmw/perception/frame_gen.hpp"

using namespace shmmw::perception;

int main(int argc, char** argv) {
    int n_frames = 200;
    if (argc > 1) n_frames = atoi(argv[1]);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("shmmw_ros2_crossing_pub");
    auto pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("/shmmw_perception/frame", 10);

    rclcpp::sleep_for(std::chrono::milliseconds(500)); // let discovery settle before the timed run

    std::vector<LidarPoint> points(kNumPoints);
    std::vector<uint8_t> camera_bg(kImgW * kImgH * 4);

    const size_t payload_bytes = sizeof(int64_t) + sizeof(uint64_t) + camera_bg.size();
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.resize(payload_bytes);

    for (int f = 0; f < n_frames; ++f) {
        generate_frame(777, f, points.data(), camera_bg.data());

        int64_t send_ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();
        uint64_t frame_idx = f;
        size_t off = 0;
        memcpy(msg.data.data() + off, &send_ts_ns, sizeof(send_ts_ns));
        off += sizeof(send_ts_ns);
        memcpy(msg.data.data() + off, &frame_idx, sizeof(frame_idx));
        off += sizeof(frame_idx);
        memcpy(msg.data.data() + off, camera_bg.data(), camera_bg.size());

        pub->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds(5));
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    rclcpp::shutdown();
    printf("published %d frames\n", n_frames);
    return 0;
}
