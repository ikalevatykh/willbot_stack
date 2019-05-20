#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/joint_limits_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float64MultiArray.h>

namespace willbot_controllers
{
class JointVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);

protected:
  // parameters
  double keep_command_duration_;
  std::vector<std::string> joint_names_;

  unsigned int n_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // bool has_limits_;
  // std::vector<joint_limits_interface::VelocityJointSoftLimitsHandle> joint_limits_;

  struct VelocityCommand
  {
    ros::Time expired;
    std::vector<double> velocities;
  };
  realtime_tools::RealtimeBuffer<VelocityCommand> command_buffer_;

  ros::Subscriber sub_command_;
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
};
}