#include "agent/agent.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("update_goal_node");

    // Usage Agent_name x y z

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x =  atof(argv[2]);
    start_pose.position.y =  atof(argv[3]);
    start_pose.position.z =  atof(argv[4]);
    start_pose.orientation.w = 1.0;

    Agent_Robot agent(node,argv[1],start_pose);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}