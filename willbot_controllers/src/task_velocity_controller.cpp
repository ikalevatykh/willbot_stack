#include <willbot_controllers/admittance_controller.h>


namespace willbot_controllers
{

  bool TaskVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    // load robot description from the parameter server
    if (!kinematic_description_.init(controller_nh))
      return false;

    return true;
  }

  void TaskVelocityController::starting(const ros::Time& time)
  {
  }

  void TaskVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
  }

  void TaskVelocityController::stopping(const ros::Time& /*time*/)
  {

  }

}

PLUGINLIB_EXPORT_CLASS(willbot_controllers::TaskVelocityController, controller_interface::ControllerBase)
