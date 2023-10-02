#pragma once

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace lidar_gazebo_plugin
{

class Lidar3dSensor : public gazebo::ModelPlugin
{
public:
  Lidar3dSensor();

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  
  void Update();
  using spinRate = std_msgs::msg::Float32;

  std::string robot_namespace_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;

  gazebo::common::Time last_sim_time_;
  gazebo::common::Time last_update_time_;
  double update_period_ms_;
  gazebo::event::ConnectionPtr update_connection_;


  gazebo::physics::JointPtr joint_;
  gazebo::common::PID pid_;
  double rate_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<spinRate>::SharedPtr rate_command_sub_;
};

}
