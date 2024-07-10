#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/msg/agent_info.hpp>
#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/get_plan.hpp>
//#include <PathPlanners/CoreComponents.hpp>
#include <PathPlanners/PathPlanningLibv2.hpp>
#include <my_robot_interfaces/srv/update_map.hpp>

#include <bits/stdc++.h>
using namespace std;


class Path_Planner {
public:
    Path_Planner(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        path_planning_service_ = node_->create_service<my_robot_interfaces::srv::GetPlan>(
            "/get_plan", std::bind(&Path_Planner::planner_get_plan, this, std::placeholders::_1, std::placeholders::_2));
        
        update_map_client_ = node_->create_client<my_robot_interfaces::srv::UpdateMap>("/update_map");

        RCLCPP_INFO(node_->get_logger(), "Motion Planner Service Ready");
        timer_ = node_->create_wall_timer(std::chrono::duration<double>(5.0) , bind(&Path_Planner::init_map, this));
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr path_planning_service_;
    rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<my_robot_interfaces::srv::UpdateMap>::SharedPtr update_map_client_;
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
                    std::cout << global_map[i][j][k] << " ";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }

    void planner_get_plan(const std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> response) {
        RCLCPP_INFO(node_->get_logger(), "Plan Request Received");

        //new planner v2


        geometry_msgs::msg::Point start;
        start.x = 0;
        start.y = 0;
        start.z = 0;

        geometry_msgs::msg::Point goal;
        goal.x = request->goal_pose.position.x;
        goal.y = request->goal_pose.position.y;
        goal.z = request->goal_pose.position.z;

        AStar astar;
        auto path = astar.get_plan(global_map,start,goal);

        response->path = path;
        for(auto point : path){
            std::cout<<point.x<<" "<<point.y<<" "<<point.z<<" ";
        }

        // old planner v1

        // std::vector<int> goal = {
        //     static_cast<int>(request->goal_pose.position.x),
        //     static_cast<int>(request->goal_pose.position.y),
        //     static_cast<int>(request->goal_pose.position.z)
        // };
        // std::vector<int> start = {0,0,0};

        // RCLCPP_INFO(node_->get_logger(), "Preparing and shipping response");

        // for(auto point : path) {
        //     geometry_msgs::msg::Point p;
        //     // print the messages onto the terminal
        //     RCLCPP_INFO(node_->get_logger(), "Path Point: %d %d %d", point[0], point[1], point[2]);
        //     p.x = point[0];
        //     p.y = point[1];
        //     p.z = point[2];
        //     response->path.push_back(p);
        // }


        // we finally update the map with the new map 
        std::vector<int> new_map_vector = this->generate_new_map(path);
        this->publish_new_map(new_map_vector);
    }

    void publish_new_map(std::vector<int> new_map) {
        RCLCPP_INFO(node_->get_logger(), "Publishing new map onto map server");
        auto request = std::make_shared<my_robot_interfaces::srv::UpdateMap::Request>();
        request->map = new_map;

        auto future = update_map_client_->async_send_request(request);
    }


    std::vector<int> generate_new_map(std::vector<geometry_msgs::msg::Point> path , bool add_path = true) {
        RCLCPP_INFO(node_->get_logger(), "Generating new map");
        auto new_global_map = global_map;
        if(add_path){
            for(auto point : path) {
                if(new_global_map[point.x][point.y][point.z] >= 1)
                    new_global_map[point.x][point.y][point.z] += 1;
            }
        
        }
        else{
            for(auto point : path) {
                if(new_global_map[point.x][point.y][point.z] > 1)
                    new_global_map[point.x][point.y][point.z] -= 1;
            }
        
        }

        std::vector<int> new_map;
        for(int i=0 ; i<map_x ; ++i){
            for(int j = 0 ; j<map_y ; ++j){
                for(int k = 0 ; k<map_z ; ++k){
                    new_map.push_back(new_global_map[i][j][k]);
                }
            }
        }

        // add in the dimensions of the map at the end 
        new_map.push_back(map_x);
        new_map.push_back(map_y);
        new_map.push_back(map_z);

        return new_map;
    }

    void print_Msg(std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Response> response){
        RCLCPP_INFO(node_->get_logger(), "Map Update Result: %d", response->result);
    }
};


