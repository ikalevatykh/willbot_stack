#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "internal/robot_model.h"
#include "internal/segment.h"


namespace willbot_controllers
{

class JointVelocityController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  JointVelocityController(){}

  virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

protected:
  RobotModel robot_model_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  realtime_tools::RealtimeBuffer<Segment> segment_buffer_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg);

};

}