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
    pub_tiles_  = create_publisher<visualization_msgs::msg::Marker>("Rviz_marker_topic/Map_Tiles", 10);
    pub_lift_  = create_publisher<visualization_msgs::msg::Marker>("Rviz_marker_topic/Map_Lift", 10);
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
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.25;
    marker.color.r = 255;
    marker.color.b = 255;
    marker.color.a = 0.5;

    auto lift = marker;
    
    lift.color.r = 13.4;
    lift.color.g = 123.4;
    lift.color.b = 56.4;

    geometry_msgs::msg::Point p;
    
    for (int i =0; i <= 10; i++)
    {
      for (int j=0; j <= 10; j++)
      {

        p.x = i;
        p.y = j;
        p.z = 0;

        if (i==5 && j==5)lift.points.push_back(p);

        else
        marker.points.push_back(p);
        
      }

      for (int j=0; j <= 10; j++)
      {
        p.x = i;
        p.y = j;
        p.z = 2;
        if (i==5 && j==5)lift.points.push_back(p);

        else
        marker.points.push_back(p);
      }

      for (int j=0; j <= 10; j++)
      {
        p.x = i;
        p.y = j;
        p.z = 4;
        if (i==5 && j==5)lift.points.push_back(p);

        else
        marker.points.push_back(p);
      }
      
    }

    pub_tiles_->publish(marker);
    pub_lift_->publish(lift);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_tiles_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_lift_;
  rclcpp::TimerBase::SharedPtr timer_;

};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}