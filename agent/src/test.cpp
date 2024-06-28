#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/srv/update_goal.hpp>

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <ctime>

using namespace std::chrono_literals;
using namespace std::placeholders;
using std::vector;

class Agent_Robot 
{
public:
     explicit Agent_Robot(std::shared_ptr<rclcpp::Node> node, std::string serial_id, geometry_msgs::msg::Pose start_pose, const double period = 5.0, const double timer_hz = 30.0)
        : node_(node), serial_id_(serial_id), pose_(start_pose), period_(period), timer_hz_(timer_hz)
     {

        int a = serial_id.at(6);
        srand(time(NULL) + a);
        agent_color[0] = (rand() % 100) * 0.01;
        agent_color[1] = (rand() % 100) * 0.01;
        agent_color[2] = (rand() % 100) * 0.01;
        dt_position_ = 0;
        done_ = true;

        pub_agent_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("Rviz_marker_topic/base_link", 100);
        service_ = node_->create_service<my_robot_interfaces::srv::UpdateGoal>("update_goal", std::bind(&Agent_Robot::agent_update_goal,this,_1,_2));
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

        timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&Agent_Robot::agent_update_pose, this));
        RCLCPP_INFO(node_->get_logger(), "Ready to update goal pose for agent");
     }

private:

    std::shared_ptr<rclcpp::Node> node_;
    std::string serial_id_;                                 
    geometry_msgs::msg::Pose pose_; 

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_agent_marker_;                     
    rclcpp::Service<my_robot_interfaces::srv::UpdateGoal>::SharedPtr service_;  
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

    const double period_;                                   // the time in seconds that it should take for the agent to traverse the path - defaults to 10
    const double timer_hz_;                                 // the frequency at which the timer should run - defaults arbitrarily to 30 Hz
    double agent_color[3];                                  // Array to hold the RGB color of the agent
    double dt_position_;                                    // increment used to update the position of the agent as it moves
    bool done_;                                             // When true, the agent has reached its goal pose
    geometry_msgs::msg::Pose goal_pose_;                    // goal pose of the agent
    vector<geometry_msgs::msg::Point> point_list_;          // list of points that compose the path

    void agent_update_pose()
    {           

        agent_update_transform(goal_pose_);
        agent_build_agent_marker();
    }
 
    void agent_update_goal(std::shared_ptr<my_robot_interfaces::srv::UpdateGoal::Request> req ,std::shared_ptr<my_robot_interfaces::srv::UpdateGoal::Response> res )
    {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Iniating Movement to " << req->goal_pose.position.x);
        goal_pose_ = req->goal_pose;
        
        dt_position_=goal_pose_.position.x-pose_.position.x;

        geometry_msgs::msg::Point send;
        send.x=goal_pose_.position.x;
        send.y=0;
        send.x=0;
        res->path.push_back(send);
    }

    void agent_update_transform(geometry_msgs::msg::Pose pose)
    {   

    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;

    q.setX(pose.orientation.x);
    q.setY(pose.orientation.y);
    q.setZ(pose.orientation.z);
    q.setW(pose.orientation.w);
    q.normalize();

    transform.header.stamp = node_->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = serial_id_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    br_->sendTransform(transform);

    }

    void agent_build_agent_marker()
    {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = serial_id_;
    marker.header.stamp = node_->now();
    marker.ns = serial_id_;
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.z = 0.25;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = agent_color[0];
    marker.color.g = agent_color[1];
    marker.color.b = agent_color[2];
    pub_agent_marker_->publish(marker);

    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("update_goal_server");

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0.0;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.0;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 1.0;

    Agent_Robot agent(node, "agent_1", start_pose);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
