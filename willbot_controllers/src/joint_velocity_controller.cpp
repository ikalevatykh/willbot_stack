#include <willbot_controllers/admittance_controller.h>


namespace willbot_controllers
{

  bool JointVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    return false;
  }

  void JointVelocityController::starting(const ros::Time& time)
  {
  }

  void JointVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
  }

  void JointVelocityController::stopping(const ros::Time& /*time*/)
  {}

}

PLUGINLIB_EXPORT_CLASS(willbot_controllers::JointVelocityController, controller_interface::ControllerBase)
