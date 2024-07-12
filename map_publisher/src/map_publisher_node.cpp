#include "rclcpp/rclcpp.hpp"

#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/update_map.hpp>

#include <vector>

using namespace std::chrono_literals; // for time literals like 1s

class MapPublisherNode : public rclcpp::Node
{
    public:
        MapPublisherNode() : Node("map_publisher_node")
        {
            RCLCPP_INFO(this->get_logger(), "Map Publisher Node initialized.");

            
            get_service_ = this->create_service<my_robot_interfaces::srv::GetMap>(
                "/get_map", std::bind(&MapPublisherNode::get_map , this, std::placeholders::_1, std::placeholders::_2)
            );

            update_service_ = this->create_service<my_robot_interfaces::srv::UpdateMap>(
                "/update_map", std::bind(&MapPublisherNode::update_map , this, std::placeholders::_1, std::placeholders::_2)
            );

            // Initialize the 3D grid
            init_map();

        }

    private:

        //service to get the map and update it 
        rclcpp::Service<my_robot_interfaces::srv::GetMap>::SharedPtr get_service_;
        rclcpp::Service<my_robot_interfaces::srv::UpdateMap>::SharedPtr update_service_;

        std::vector<int> global_map;

        // Initialize the grid map
        void init_map()
        {
                // Define the three 10x10 matrices
            int data1[10][10] = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, -1, 1, 1, 0, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
                {1, 1, 0, 1, -1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 0, 1, 1, 0, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, 1, 1, 0},
                {0, 1, 0, 1, -1, 1, 1, -1, 1, 1},
                {0, 0, 0, 0, 1, 0, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 0, 0, 1}
            };

            int data2[10][10] = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 0, -1, 0, 1, 1, 1, 0},
                {1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, -1, 1, 1, 1, 1, 0},
                {1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
                {1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
                {1, 1, 1, 0, -1, 0, 0, -1, 0, 1},
                {1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
            };

            int data3[10][10] = {
                {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, -1, 1, 1, 1, 0, 0},
                {0, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                {0, 1, 1, 1, 0, 0, 0, 0, 1, 1},
                {1, 1, 1, 1, -1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 0, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, -1, 1, 0, -1, 0, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {0, 0, 1, 1, 1, 1, 1, 1, 1, 1}
            };
            
            // Append data1
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data1[i][j]);
                }
            }
            
            // Append data2
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data2[i][j]);
                }
            }
            
            // Append data3
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data3[i][j]);
                }
            }

            // Append the dimensions of the matrices
            global_map.push_back(3);
            global_map.push_back(10);
            global_map.push_back(10);
        }

        // service to get map 
        void get_map(const std::shared_ptr<my_robot_interfaces::srv::GetMap::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::GetMap::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request");

            // Send the map
            response->map = global_map;

            RCLCPP_INFO(this->get_logger(), "Map sent");
        }

        //service to update it 
        void update_map(const std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request");

            // Update the map
            global_map = request->map;

            // Send response
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "Map updated");
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>(); // Example: 10x10x4 grid
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
