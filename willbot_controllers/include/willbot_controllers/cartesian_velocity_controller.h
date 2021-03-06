#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>

namespace willbot_controllers
{
class CartesianVelocityController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& /*root_nh*/,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);

protected:
  // parameters
  double keep_command_duration_;
  std::string base_frame_id_;
  std::string tool_frame_id_;

  unsigned int n_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // bool has_limits_;
  // std::vector<joint_limits_interface::VelocityJointSoftLimitsHandle> joint_limits_;

  std::unique_ptr<KDL::ChainIkSolverVel> ik_vel_;
  std::unique_ptr<KDL::ChainFkSolverPos> fk_pos_;
  KDL::JntArray current_q_;
  KDL::Frame current_x_;
  KDL::JntArray desired_qdot_;
  KDL::Twist desired_v_;

  struct TwistCommand
  {
    ros::Time expiring;
    KDL::Twist twist;
    bool tool;
  };
  realtime_tools::RealtimeBuffer<TwistCommand> command_buffer_;

  ros::Subscriber sub_command_;
  void commandCB(const geometry_msgs::TwistStampedConstPtr& msg);
};
}
