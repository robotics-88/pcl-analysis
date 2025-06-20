/*
© 2023 Robotics 88
Author: Gus Meyer <gus@robotics88.com>
*/

#include "pcl_analysis/pcl_analysis.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PCLAnalysis>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}