#include "agent/agent.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("update_goal_node");

    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.w = 1.0;

    Agent_Robot agent(node, "agent_1", start_pose);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
