#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <my_robot_interfaces/msg/
#include <geometry_msgs/msg/point.hpp>

#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/get_plan.hpp>
#include <PathPlanners/CoreComponents.hpp>

#include <bits/stdc++.h>
using namespace std;


class Path_Planner{

    public : 
        Path_Planner(std::shared_ptr<rclcpp::Node> node, const int period = 10) : node_(node),period_(period)
        {
            // RCLCPP_INFO(node_->get_logger(), "This is %d", PI);
            service_ = node->create_service<my_robot_interfaces::srv::GetPlan>("/get_plan", std::bind(&Path_Planner::planner_get_plan,this,_1,_2));
            client_ = node->create_client<my_robot_interfaces::srv::GetMap>("/get_map");
            RCLCPP_INFO(node->get_logger(), "Motion Planner Service Ready");
        }
        
    private :

        std::shared_ptr<rclcpp::Node> node_;                                                                       
        rclcpp::Service<my_robot_interfaces::srv::GetPlan>::SharedPtr service_;  

        // a client to get the map 
        rclcpp::Client<my_robot_interfaces::srv::GetMap>::SharedPtr client_;

        vector<vector<vector<int>>> global_map;

        int map_x , map_y , map_z;


        void init_map()
        {
            auto request = std::make_shared<my_robot_interfaces::srv::GetMap::Request>();
            auto result = client_->async_send_request(request);
            rclcpp::spin_until_future_complete(node_, result);
            vector<int> tempo = result.get()->map;

            update_map(tempo);
            RCLCPP_INFO(node_->get_logger(), "Map Initialized");
            print_map();
        }

        void update_map(vector<int> map)
        {
            int n = map.size();
            int map_x = map[n-3];
            int map_y = map[n-2];
            int map_z = map[n-1];

            for(int i = 0 ; i < map_x ; i++)
            {
                for(int j = 0 ; j < map_y ; j++)
                {
                    for(int k = 0 ; k < map_z ; k++)
                    {
                        global_map[i][j][k] = map[i*map_y*map_z + j*map_z + k];
                    }
                }
            }
        }

        void print_map(){
            for(int i = 0 ; i < map_x ; i++)
            {
                for(int j = 0 ; j < map_y ; j++)
                {
                    for(int k = 0 ; k < map_z ; k++)
                    {
                        cout << global_map[i][j][k] << " ";
                    }
                    cout << endl;
                }
                cout << endl;
            }
        }  

        void planner_get_plan(const std::shared_ptr<my_robot_interfaces::srv::GetPlan::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::GetPlan::Response> response)
        {
            RCLCPP_INFO(node_->get_logger(), "Plan Request Received");
            init_map();
            response->path = {1,2,3,4,5,6,7,8,9,10};
        }
}



