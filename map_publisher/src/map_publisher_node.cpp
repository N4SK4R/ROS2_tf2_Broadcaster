#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/node_status.hpp" // Update with correct message header
#include <my_robot_interfaces/srv/get_map.hpp>

#include <vector>

using namespace std::chrono_literals; // for time literals like 1s

class MapPublisherNode : public rclcpp::Node
{
public:
    MapPublisherNode(int x, int y, int z) : Node("map_publisher_node"), num_levels_x(x), num_levels_y(y), num_levels_z(z)
    {
        RCLCPP_INFO(this->get_logger(), "Map Publisher Node initialized.");

        // Create publisher for NodeStatus message on /mapp topic
        publisher_ = this->create_publisher<my_robot_interfaces::msg::NodeStatus>("/mapp", 10);
        service_ = this->create_service<my_robot_interfaces::srv::GetMap>(
            "/update_map", std::bind(&MapPublisherNode::get_map , this, std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize the 3D grid
        init_map();

        // Example: Publish the initial state of the map
        publish_map();

        // Set up publishing timer to trigger publish_map() every 1 second
        publish_timer_ = this->create_wall_timer(1s, std::bind(&MapPublisherNode::publish_map, this));
    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::NodeStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    //service to get the map and update it 
    rclcpp::Service<my_robot_interfaces::srv::GetMap>::SharedPtr service_;

    int num_levels_x;
    int num_levels_y;
    int num_levels_z;

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
        global_map.push_back(10);
        global_map.push_back(10);
        global_map.push_back(3);
    }

    // Publishes the current state of the map
    void publish_map()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing map...");

        my_robot_interfaces::msg::NodeStatus msg;
        msg.map_data = global_map;

        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Map published.");
    }

    //service to get the map and update it
    void get_map(const std::shared_ptr<my_robot_interfaces::srv::GetMap::Request> request,
        std::shared_ptr<my_robot_interfaces::srv::GetMap::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Incoming request");

        // Update the map
        global_map = request->map;

        // Send response
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Map updated");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>(10, 10, 4); // Example: 10x10x4 grid
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
