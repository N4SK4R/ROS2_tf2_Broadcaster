#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>


using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("Rviz_marker_publisher")
  {
    pub_  = create_publisher<visualization_msgs::msg::Marker>("Rviz_marker_topic/Map_nodes", 10);
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Publishing Rviz Markers");
  }

  void timerCallback()
  {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    marker.ns = "grid_nodes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    

    for (size_t i =0; i <= 10; i++)
        for (size_t j=0; j <= 10; j++)
        {
            geometry_msgs::msg::Point p;
            p.x = i;
            p.y = j;
            marker.points.push_back(p);
        }
    pub_->publish(marker);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}