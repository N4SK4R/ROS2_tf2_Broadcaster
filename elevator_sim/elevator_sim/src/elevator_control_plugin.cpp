#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>

namespace gazebo
{
class ElevatorControlPlugin : public ModelPlugin
{
public:
  ElevatorControlPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Get the joint name from SDF
    std::string joint_name = sdf->GetElement("joint_name")->Get<std::string>();

    // Get the joint pointer
    joint_ = model->GetJoint(joint_name);
    if (!joint_)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint %s not found", joint_name.c_str());
      return;
    }

    // Create subscriber for target position
    target_position_sub_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
      "target_position", 10,
      std::bind(&ElevatorControlPlugin::targetPositionCallback, this, std::placeholders::_1));

    // Connect to the world update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ElevatorControlPlugin::OnUpdate, this));

   
    kp_ = 500.0;  
    ki_ = 50.0;   
    kd_ = 100.0;  
    integral_ = 0.0;
    prev_error_ = 0.0;
    max_force_ = 1000.0;  

    last_update_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(ros_node_->get_logger(), "Elevator control plugin loaded successfully");
  }

  void OnUpdate()
  {
    if (target_position_set_)
    {
      auto current_time = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(current_time - last_update_time_).count();
      last_update_time_ = current_time;

      double current_position = joint_->Position(0);
      double error = target_position_ - current_position;

      // PID control
      integral_ += error * dt;
      double derivative = (error - prev_error_) / dt;
      
      double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

      // Clamp the output force
      double force = std::max(-max_force_, std::min(output, max_force_));
      
      joint_->SetForce(0, force);

      prev_error_ = error;

      // Debug output
      RCLCPP_DEBUG(ros_node_->get_logger(), "Current pos: %.3f, Target: %.3f, Force: %.3f", 
                   current_position, target_position_, force);
    }
  }

private:
  void targetPositionCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    target_position_ = static_cast<double>(msg->data) / 100.0; // Convert cm to meters
    target_position_set_ = true;
    // Reset integral term when new target is set
    integral_ = 0.0;
    RCLCPP_INFO(ros_node_->get_logger(), "New target position received: %.2f m", target_position_);
  }

  physics::JointPtr joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_position_sub_;
  event::ConnectionPtr update_connection_;
  double target_position_ = 0.0;
  bool target_position_set_ = false;

  // PID controller variables
  double kp_, ki_, kd_;
  double integral_;
  double prev_error_;
  double max_force_;
  std::chrono::steady_clock::time_point last_update_time_;
};

GZ_REGISTER_MODEL_PLUGIN(ElevatorControlPlugin)
}