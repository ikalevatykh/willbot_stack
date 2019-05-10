#include <willbot_controllers/admittance_controller.h>

#include <kdl_conversions/kdl_msg.h>

namespace willbot_controllers
{
bool AdmittanceController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh)
{
  // load kinematic description from parameter server
  if (!kinematic_description_.init(controller_nh))
    return false;

  // init force torque sensor subscriber
  // TODO: switch to the multi-hw interface
  if (!ft_sensor_.init(controller_nh))
    return false;    

  for (const auto joint_name : kinematic_description_.joint_names())
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_name));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }
  n_joints_ = joints_.size();

  force_torque_ = ft_sensor_.getHandle();

  // init admittance
  admittance_.init(controller_nh, kinematic_description_.chain(), force_torque_.getFrameId());

  q_goal_ = KDL::JntArray(n_joints_);
  q_curr_ = KDL::JntArray(n_joints_);
  return true;
}

void AdmittanceController::starting(const ros::Time& time)
{
  ft_sensor_.starting();

  ros::Duration(1.0).sleep();

  ft_sensor_.update(time);

  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    q_goal_(i) = joints_[i].getPosition();
    q_curr_(i) = joints_[i].getPosition();
  }

  for (unsigned int i = 0; i < 3; ++i)
  {
    w_goal_.force(i) = force_torque_.getForce()[i];
    w_goal_.torque(i) = force_torque_.getTorque()[i];
  }

  admittance_.reset(q_goal_);
}

void AdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
  // read current position
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    q_curr_(i) = joints_[i].getPosition();
  }

  // read force torque
  ft_sensor_.update(time);
  for (unsigned int i = 0; i < 3; ++i)
  {
    w_curr_.force(i) = force_torque_.getForce()[i];
    w_curr_.torque(i) = force_torque_.getTorque()[i];
  }

  // update admittance internal state
  admittance_.update(q_goal_, q_curr_, w_goal_, w_curr_, period.toSec());

  // update velocity command
  // const auto& commands = admittance_.command_velocity();

  // write command velocities
  for (unsigned int i = 0; i < n_joints_; i++)
  {
    // joints_[i].setCommand(commands[i]);
  }
}

void AdmittanceController::stopping(const ros::Time& /*time*/)
{
  ft_sensor_.stopping();
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::AdmittanceController, controller_interface::ControllerBase)
