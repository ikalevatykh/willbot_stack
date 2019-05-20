#include <willbot_controllers/cartesian_velocity_controller.h>
#include <willbot_controllers/force_torque_limited_controller.h>
#include <willbot_controllers/joint_velocity_controller.h>
#include <willbot_controllers/utils.h>

#include <kdl/frames_io.hpp>

namespace willbot_controllers
{
template <class TController>
bool ForceTorqueLimitedController<TController>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh)
{
  auto* joints_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
  auto* sensor_hw = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();

  // Force torque sensor name
  if (!nh.getParam("force_torque_sensor", sensor_name_))
  {
    ROS_ERROR_STREAM("Could not find 'force_torque_sensor' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Sensor handle
  try
  {
    sensor_handle_ = sensor_hw->getHandle(sensor_name_);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Exception thrown: " << e.what());
    return false;
  }

  // Joint handles
  const auto& joint_names = joints_hw->getNames();
  for (const auto& joint_name : joint_names)
  {
    joint_handles_.push_back(joints_hw->getHandle(joint_name));
  }

  // Velocity controller
  if (!controller_.init(joints_hw, nh))
  {
    return false;
  }

  force_limit_abs_ = 10.0;
  force_limit_rel_ = 10.0;
  return true;
}

template <class TController>
void ForceTorqueLimitedController<TController>::starting(const ros::Time& time)
{
  controller_.starting(time);

  utils::getWrench(sensor_handle_, wrench_bias_);
  wrench_previous_ = wrench_bias_;
  running_ = true;
}

template <class TController>
void ForceTorqueLimitedController<TController>::update(const ros::Time& time, const ros::Duration& period)
{
  if (!running_)
  {
    utils::zeroCommand(joint_handles_);
    return;
  }

  utils::getWrench(sensor_handle_, wrench_current_);

  if ((wrench_current_ - wrench_bias_).force.Norm() > force_limit_abs_)
  {
    ROS_DEBUG_STREAM("Abs force violated: " << (wrench_current_ - wrench_bias_));
    running_ = false;
  }

  if ((wrench_current_ - wrench_previous_).force.Norm() > force_limit_rel_)
  {
    ROS_DEBUG_STREAM("Rel force violated: " << (wrench_current_ - wrench_previous_));
    running_ = false;
  }

  wrench_previous_ = wrench_current_;

  controller_.update(time, period);
}

template <class TController>
void ForceTorqueLimitedController<TController>::stopping(const ros::Time& time)
{
  controller_.stopping(time);
}

// Exporting types
typedef ForceTorqueLimitedController<CartesianVelocityController> CartesianVelocityForceTorqueLimitedController;
typedef ForceTorqueLimitedController<JointVelocityController> JointVelocityForceTorqueLimitedController;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::CartesianVelocityForceTorqueLimitedController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(willbot_controllers::JointVelocityForceTorqueLimitedController,
                       controller_interface::ControllerBase)