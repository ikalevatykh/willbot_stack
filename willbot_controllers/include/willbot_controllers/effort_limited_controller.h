#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/stiffness.hpp>

#include <willbot_controllers/admittance_paramConfig.h>

namespace willbot_controllers
{
template <class TController>
class ForceTorqueLimitedController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                            hardware_interface::ForceTorqueSensorInterface>
{
public:
  virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);
  virtual void stopping(const ros::Time& time);

protected:
  std::string sensor_name_;
  hardware_interface::ForceTorqueSensorHandle sensor_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  TController controller_;

  double force_limit_abs_;
  double force_limit_rel_;
  bool limits_violated_;

  KDL::Wrench wrench_bias_;
  KDL::Wrench wrench_previous_;
  KDL::Wrench wrench_current_;
};
}