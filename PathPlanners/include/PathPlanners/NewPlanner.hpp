#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/msg/agent_info.hpp>
#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/get_plan.hpp>
//#include <PathPlanners/CoreComponents.hpp>
#include <PathPlanners/PathPlanningLib.hpp>
#include <bits/stdc++.h>
using namespace std;


class Path_Planner {
public:
    Path_Planner(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        path_planning_service_ = node_->create_service<my_robot_interfaces::srv::GetPlan>(
            "/get_plan", std::bind(&Path_Planner::planner_get_plan, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "Motion Planner Service Ready");
        timer_ = node_->create_wall_timer(std::chrono::duration<double>(5.0) , bind(&Path_Planner::init_map, this));
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr path_planning_service_;
    rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<vector<vector<int>>> global_map;
    int map_x, map_y, map_z;
    bool map_is_initialized;

    void init_map() {
            RCLCPP_INFO(node_->get_logger(), "Initializing Map");
            get_map_client_ = node_->create_client<my_robot_interfaces::srv::GetMap>("/get_map");

            auto request = std::make_shared<my_robot_interfaces::srv::GetMap::Request>();
            auto future_result = get_map_client_->async_send_request(request,
                std::bind(&Path_Planner::responseCallback, this, std::placeholders::_1));
            // Set flag here; responseCallback will finalize initialization
    }

    void responseCallback(rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedFuture future) {
        RCLCPP_INFO(node_->get_logger(), "Fresh map recieved");
        auto result = future.get();
        map_x = result->map[result->map.size() - 3];
        map_y = result->map[result->map.size() - 2];
        map_z = result->map[result->map.size() - 1]; 

        global_map.resize(map_x, vector<vector<int>>(map_y, vector<int>(map_z)));

        update_map(result->map);
    }

    void update_map(const vector<int>& map) {
        RCLCPP_INFO(node_->get_logger(), "Updating Map");

        for(int i = 0 ; i < map_x ; i++) {
            for(int j = 0 ; j < map_y ; j++) {
                for(int k = 0 ; k < map_z ; k++) {
                    global_map[i][j][k] = map[i * map_y * map_z + j * map_z + k];
                }
            }
        }
        RCLCPP_INFO(node_->get_logger(),"Map Dimensions: %d %d %d", map_x, map_y, map_z);
        //print_map();
    }

    void print_map() {
        RCLCPP_INFO(node_->get_logger(), "Map Contents:");

        for(int i = 0 ; i < map_x ; i++) {
            for(int j = 0 ; j < map_y ; j++) {
                for(int k = 0 ; k < map_z ; k++) {
                    RCLCPP_INFO(node_->get_logger(), "%d ", global_map[i][j][k]);
                }
                RCLCPP_INFO(node_->get_logger(), "\n");
            }
            RCLCPP_INFO(node_->get_logger(), "\n");
        }
    }

    void planner_get_plan(const std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> response) {
        RCLCPP_INFO(node_->get_logger(), "Plan Request Received");

        std::vector<int> goal = {
            static_cast<int>(request->goal_pose.position.x),
            static_cast<int>(request->goal_pose.position.y),
            static_cast<int>(request->goal_pose.position.z)
        };
        std::vector<int> start = {0,0,0};

        Astar astar(global_map, start, goal);
        auto path = astar.get_map();

        // Do planning here

        RCLCPP_INFO(node_->get_logger(), "Preparing and shipping response");

        for(auto point : path) {
            geometry_msgs::msg::Point p;
            // print the messages onto the terminal
            RCLCPP_INFO(node_->get_logger(), "Path Point: %d %d %d", point[0], point[1], point[2]);
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            response->path.push_back(p);
        }
    }
};


