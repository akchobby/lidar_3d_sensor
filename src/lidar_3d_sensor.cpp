// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <iostream>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include "lidar_3d_sensor.hpp"

namespace lidar_gazebo_plugin
{

Lidar3dSensor::Lidar3dSensor()
: robot_namespace_{""},
  last_sim_time_{0},
  last_update_time_{0},
  update_period_ms_{8}

{
}

void Lidar3dSensor::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get model and world references
  model_ = model;
  world_ = model_->GetWorld();
  auto physicsEngine = world_->Physics();
  physicsEngine->SetParam("friction_model", std::string{"cone_model"});

  if (sdf->HasElement("robotNamespace")) {
    robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  // Default to 5 rad/s velocity
  rate_ = 5.0;

  // Check that the velocity element exists, then read the value
  if (sdf->HasElement("velocity"))
    rate_ = sdf->GetElement("velocity")->Get<double>();

  // Set up ROS node and subscribers and publishers
  ros_node_ = gazebo_ros::Node::Get(sdf);
  RCLCPP_INFO(ros_node_->get_logger(), "Loading lidar 3D Gazebo Plugin");

  //  joints, asumming the lidar has only one joint
  joint_ = model_->GetJoints()[0];
  RCLCPP_DEBUG(ros_node_->get_logger(), joint_->GetScopedName().c_str());


  // Setup a P-controller, with a gain of 0.1.
  pid_ = gazebo::common::PID(0.1, 0, 0);

  model_->GetJointController()->SetVelocityPID(
    joint_->GetScopedName(), pid_);

  // Set the joint's target velocity. default val

  RCLCPP_INFO(ros_node_->get_logger(), joint_->GetScopedName().c_str());

  rate_command_sub_ = ros_node_->create_subscription<spinRate>(
    "/lidar_spin_rate",
    10,
    [ = ](std::shared_ptr<spinRate> rate) {
      rate_ = rate->data;
      model_->GetJointController()->SetVelocityTarget(
      joint_->GetScopedName(), rate_);
      RCLCPP_INFO(ros_node_->get_logger(), "rate set to %f", rate_);
    }
  );

   RCLCPP_INFO(ros_node_->get_logger(), "velocity curr: %f",joint_->GetVelocity(1));

   update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&Lidar3dSensor::Update, this));

}


void Lidar3dSensor::Update()
{
  auto cur_time = world_->SimTime();
  if (last_sim_time_ == 0) {
    last_sim_time_ = cur_time;
    last_update_time_ = cur_time;
    return;
  }

  auto dt = (cur_time - last_sim_time_).Double();

  // Update joint PID

  auto error = joint_->GetVelocity(0)- rate_;

  auto force = pid_.Update(error, dt);
  // RCLCPP_INFO(ros_node_->get_logger(), "Error curr: %f",error);

  joint_->SetForce(0, force);
  // RCLCPP_INFO(ros_node_->get_logger(), "velocity curr: %f",joint_->GetVelocity(0));

}



GZ_REGISTER_MODEL_PLUGIN(Lidar3dSensor)

} 
