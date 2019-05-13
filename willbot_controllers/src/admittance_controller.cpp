#include <willbot_controllers/admittance_controller.h>
#include <willbot_controllers/internal/utils.h>

namespace willbot_controllers
{
bool AdmittanceController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh)
{
  // load kinematic description from parameter server
  if (!robot_model_.init(controller_nh))
    return false;

  // claim joint handles 
  for (const auto joint_name : robot_model_.joint_names())
  {
    try
    {
      joint_handles_.push_back(hw->getHandle(joint_name));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }
  n_joints_ = joint_handles_.size();

  // init force torque sensor subscriber
  // We use this wrapper class because ur_modern_driver 
  // does not support multi hardware interface
  if (!ft_sensor_.init(controller_nh))
    return false; 

  // claim force torque handle 
  try
  {
    force_torque_hadle_ = ft_sensor_.getHandle();
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Exception thrown: " << e.what());
    return false;
  }

  // init admittance
  admittance_.init(controller_nh, robot_model_.chain(), force_torque_hadle_.getFrameId());

  // reserve memory
  q_goal_ = KDL::JntArray(n_joints_);
  q_curr_ = KDL::JntArray(n_joints_);
  return true;
}

void AdmittanceController::starting(const ros::Time& time)
{
  // read latest force torque message
  ft_sensor_.update(time);

  // update current position
  get_position(joint_handles_, q_curr_);

  // update current wrench
  get_wrench(force_torque_hadle_, w_curr_);

  // TODO: apply fts compensation

  // set current values as admittance equilibrium point 
  admittance_.reset(q_curr_, w_curr_);
}

void AdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
  // read latest force torque message
  ft_sensor_.update(time);

  // update current position
  get_position(joint_handles_, q_curr_);

  // update current wrench
  get_wrench(force_torque_hadle_, w_curr_);

  // TODO: apply fts compensation

  // update admittance internal state
  admittance_.update(q_curr_, w_curr_, period.toSec());

  // write velocity
  set_command(admittance_.output(), joint_handles_);
}

void AdmittanceController::stopping(const ros::Time& /*time*/)
{
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::AdmittanceController, controller_interface::ControllerBase)
