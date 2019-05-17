#include <willbot_controllers/admittance_controller.h>

namespace willbot_controllers
{
bool JointVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  return false;
}

void JointVelocityController::starting(const ros::Time& time)
{
  segment_buffer_.initRT()
}

void JointVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  const auto& segment = *segment_buffer_.readFromNonRT();
}

void JointVelocityController::stopping(const ros::Time& /*time*/)
{
}

void TaskVelocityController::commandCB(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
{
  if (msg->positions.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of positions (" << msg->positions.size() << ") does not match number of joints ("
                                                << n_joints_ << ")! Not executing!");
    return;
  }

  if (msg->velocities.size() != n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of velocities (" << msg->velocities.size() << ") does not match number of joints ("
                                                 << n_joints_ << ")! Not executing!");
    return;
  }

  if (msg->accelerations.size() != 0)
  {
    ROS_WARN_STREAM("Accelerations will be ignored!");
    return;
  }

  if (msg->effort.size() != 0)
  {
    ROS_WARN_STREAM("Effort will be ignored!");
    return;
  }

  const auto& segment_current = *segment_buffer_.readFromNonRT();

  Segment segment_new();

  commands_buffer_.writeFromNonRT(segment_new);

  // float64[] positions
  // float64[] velocities
  // float64[] accelerations
  // float64[] effort
  // duration time_from_start

  // commands_buffer_.writeFromNonRT(msg->data);
}
}

PLUGINLIB_EXPORT_CLASS(willbot_controllers::JointVelocityController, controller_interface::ControllerBase)
