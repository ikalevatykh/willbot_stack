#include <willbot_controllers/admittance_controller.h>


namespace willbot_controllers
{

  bool AdmittanceController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh)
  {
    return false;
  }

  void AdmittanceController::starting(const ros::Time& time)
  {
  }

  void AdmittanceController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
  }

  void AdmittanceController::stopping(const ros::Time& /*time*/)
  {}

}

PLUGINLIB_EXPORT_CLASS(willbot_controllers::AdmittanceController, controller_interface::ControllerBase)
