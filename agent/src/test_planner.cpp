#include "agent/planner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Motion_Planner_server");

    Motion_Planner agent(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
