#include <willbot_controllers/joint_velocity_controller.h>

namespace willbot_controllers
{
bool JointVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controller_nh)
{
  ros::NodeHandle& nh = controller_nh;

  // Period for keeping previous command
  if (!nh.getParam("keep_command_duration", keep_command_duration_))
  {
    ROS_ERROR_STREAM("Could not find 'keep_command_duration' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // List of controlled joints
  if (!nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Could not find 'joints' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }
  n_joints_ = joint_names_.size();

  // Joint handle
  for (const auto& joint_name : joint_names_)
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

  // Joint limits
  // has_limits_ = false;
  // TODO: fill joint_limits_

  sub_command_ = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointVelocityController::commandCB, this);
  return true;
}

void JointVelocityController::starting(const ros::Time& time)
{
  const auto command = command_buffer_.readFromRT();
  command->velocities.assign(n_joints_, 0.0);
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& period)
{
  const auto command = command_buffer_.readFromRT();
  const bool expired = time >= command->expired;

  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    const double vel = expired ? 0.0 : command->velocities[i];

    joint_handles_[i].setCommand(vel);
    // if (has_limits_)
    //   joint_limits_[i].enforceLimits(period);
  }
}

void JointVelocityController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  if (msg->data.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of velocities (" << msg->data.size() << ") does not match number of joints ("
                                                 << n_joints_ << ")! Not executing!");
    return;
  }

  const auto expired = ros::Time::now() + ros::Duration(keep_command_duration_);
  command_buffer_.writeFromNonRT({ expired, msg->data });
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::JointVelocityController, controller_interface::ControllerBase)
