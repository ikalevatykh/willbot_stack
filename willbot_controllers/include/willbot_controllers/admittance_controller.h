#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "internal/admittance.h"
#include "internal/force_torque_sensor_interface.h"
#include "internal/kinematic_description.h"

namespace willbot_controllers
{
class AdmittanceController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  AdmittanceController()
  {
  }

  virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  ForceTorqueSensorInterface ft_sensor_;
  RobotModel robot_model_;
  Admittance admittance_;

  hardware_interface::ForceTorqueSensorHandle force_torque_hadle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  unsigned int n_joints_;

  KDL::JntArray q_goal_;
  KDL::JntArray q_curr_;
  KDL::Wrench w_goal_;
  KDL::Wrench w_curr_;
};
}