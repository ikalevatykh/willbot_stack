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
class CartesianAdmittanceController
    : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                            hardware_interface::ForceTorqueSensorInterface>
{
public:
  virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);

protected:
  hardware_interface::ForceTorqueSensorHandle sensor_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  unsigned int n_joints_;

  std::string base_frame_id_;
  std::string tool_frame_id_;
  std::string sensor_name_;

  std::unique_ptr<KDL::ChainFkSolverPos> fk_pos_;
  std::unique_ptr<KDL::ChainIkSolverVel> ik_vel_;
  KDL::Frame fts_to_tool_;

  KDL::Frame x_equilibrium_;
  KDL::JntArray q_equilibrium_;
  KDL::Wrench wrench_bias_;

  KDL::Wrench wrench_current_;
  KDL::Frame x_current_;
  KDL::JntArray q_current_;
  KDL::Twist twist_desired_;
  KDL::JntArray qdot_desired_;

  // Dynamic reconfigure
  std::vector<bool> axis_enabled_;
  KDL::Stiffness k_gain_;
  KDL::Stiffness d_gain_;
  KDL::Stiffness m_gain_;

  std::unique_ptr<dynamic_reconfigure::Server<willbot_controllers::admittance_paramConfig>>
      dynamic_server_admittance_param_;
  void admittanceParamCallback(willbot_controllers::admittance_paramConfig& config, uint32_t /*level*/);
};
}