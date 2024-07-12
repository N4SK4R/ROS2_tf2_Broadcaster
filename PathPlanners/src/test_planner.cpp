#include "PathPlanners/planner.hpp"
#include "PathPlanners/NewPlanner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Motion_Planner_server");

    //Path_Planner PathPlanner(node);
    Path_Planner PathPlanner(node,10);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
