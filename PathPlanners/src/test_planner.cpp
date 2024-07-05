#include "PathPlanners/planner.hpp"
#include "PathPlanners/NewPlanner.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Motion_Planner_server");

    Path_Planner agent(node);
    //Motion_Planner agent(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
