/*
© 2024 Robotics 88
Author: Erin Linebarger <erin@robotics88.com>
*/

#include "pcl_analysis/trail.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<trail::Trail>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}