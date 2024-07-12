#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <my_robot_interfaces/srv/update_goal.hpp>
#include <my_robot_interfaces/srv/get_plan.hpp>
#include <my_robot_interfaces/msg/agent_info.hpp>

#include <vector>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;
using std::vector;

class Agent_Robot 
{
public:
    explicit Agent_Robot(std::shared_ptr<rclcpp::Node> node, std::string serial_id, geometry_msgs::msg::Pose start_pose, const double period = 10.0, const double timer_hz = 30.0)
        : node_(node), serial_id(serial_id), pose(start_pose), period(period), timer_hz(timer_hz)
    {
        agent_color[0] = 188;
        agent_color[1] = 215;
        agent_color[2] = 224;
        dt_position = 0;
        done = true;
        teleport=false;

        pub_agent_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("/Rviz_marker_topic/base_link", 10);
        pub_path_marker_  = node_->create_publisher<visualization_msgs::msg::Marker>("/Rviz_marker_topic/path", 10);
        pub_agent_info_   = node_->create_publisher<my_robot_interfaces::msg::AgentInfo>("/agent_info", 10);
        service_ = node_->create_service<my_robot_interfaces::srv::UpdateGoal>(serial_id+"/update_goal", std::bind(&Agent_Robot::agent_update_goal,this,_1));
        get_plan_service_client_ = node_->create_client<my_robot_interfaces::srv::GetPlan>("/get_plan");
        br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

        timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0/timer_hz), std::bind(&Agent_Robot::agent_update_pose, this));
        RCLCPP_INFO(node_->get_logger(), "Ready to update goal pose for %s", serial_id.c_str());
    }

private:

    std::shared_ptr<rclcpp::Node> node_;
    std::string serial_id;                                 
    geometry_msgs::msg::Pose pose; 

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_agent_marker_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_path_marker_;  
    rclcpp::Publisher<my_robot_interfaces::msg::AgentInfo>::SharedPtr pub_agent_info_;
    rclcpp::Client<my_robot_interfaces::srv::GetPlan>::SharedPtr get_plan_service_client_;                    
    rclcpp::Service<my_robot_interfaces::srv::UpdateGoal>::SharedPtr service_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

    const double period;     
    const double timer_hz;                                 
    double agent_color[3];                                 
    double dt_position;                                    
    bool done;   
    bool teleport;                                           
    geometry_msgs::msg::Pose goal_pose;                    
    vector<geometry_msgs::msg::Point> point_list;          

    void agent_update_pose()
    {   
        static int cntr=0, index =0;

        if (!done)
        {   
            pose.position.x += (point_list.at(index + 1).x - point_list.at(index).x)*dt_position;
            pose.position.y += (point_list.at(index + 1).y - point_list.at(index).y)*dt_position;
            pose.position.z += (point_list.at(index + 1).z - point_list.at(index).z)*dt_position;

            if (cntr*dt_position >= 1) //for every 1 segment
            {
                cntr = 0;
                index++;
                pose.position.x = point_list.at(index).x;
                pose.position.y = point_list.at(index).y;
                pose.position.z = point_list.at(index).z;
            }

            cntr++;

            if (index == int(point_list.size()-1))
            {
                done = true;
                //teleport=true;
                index = 0;
                RCLCPP_INFO(node_->get_logger(),"Target goal has been reached by %s", serial_id.c_str());
            }

        }

        my_robot_interfaces::msg::AgentInfo msg;
        msg.serial_id = serial_id;
        msg.pose = pose;
        pub_agent_info_->publish(msg);
        agent_update_transform(pose);
        agent_build_agent_marker();

        if (teleport)
        {
            pose.position.x = 5;
            pose.position.y = 5;
            pose.position.z = 2;
        }
    }
 
    void agent_update_goal(std::shared_ptr<my_robot_interfaces::srv::UpdateGoal::Request> req)
    {
        goal_pose = req->goal_pose;
        RCLCPP_INFO(node_->get_logger(),"Intiating Movement to (%f,%f,%f)", goal_pose.position.x,goal_pose.position.y,goal_pose.position.z);
        
        auto request =std::make_shared<my_robot_interfaces::srv::GetPlan::Request>();
        request->serial_id=serial_id;
        request->goal_pose=goal_pose;

        auto result= get_plan_service_client_->async_send_request(request,std::bind(&Agent_Robot::responesCallback, this,_1));

    }

    void responesCallback(rclcpp::Client<my_robot_interfaces::srv::GetPlan>::SharedFuture future)
    {
        if(future.valid())
        {
            point_list=future.get()->path;

            int segments=0;
            segments = point_list.size() - 1;
            dt_position = (segments/period)/timer_hz;
            RCLCPP_INFO(node_->get_logger(), "Path Returned with Segments %d",segments); 
            done = false;           
            agent_build_path_marker(future.get()->path); 
        }
        
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
        transform.child_frame_id = serial_id;
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br_->sendTransform(transform);
    }

    void agent_build_agent_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = serial_id;
        marker.header.stamp = node_->now();
        marker.ns = serial_id;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.z = 0.25;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = agent_color[0];
        marker.color.g = agent_color[1];
        marker.color.b = agent_color[2];
        pub_agent_marker_->publish(marker);
    }

    void agent_build_path_marker(vector<geometry_msgs::msg::Point> vect)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = node_->now();
        marker.ns = serial_id;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.color.r = agent_color[0];
        marker.color.g = agent_color[1];
        marker.color.b = agent_color[2];
        marker.color.a = 1.0;
        marker.points = vect;
        pub_path_marker_->publish(marker);
    }

};