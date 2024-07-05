#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/node_status.hpp" // Update with correct message header
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

    int num_levels_x;
    int num_levels_y;
    int num_levels_z;

    std::vector<int> global_map;

    // Initialize the grid map
    void init_map()
    {
        global_map.resize(num_levels_x * num_levels_y * num_levels_z + 3, 1);
        global_map[global_map.size() - 3] = num_levels_x;
        global_map[global_map.size() - 2] = num_levels_y;
        global_map[global_map.size() - 1] = num_levels_z;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>(10, 10, 4); // Example: 10x10x4 grid
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
