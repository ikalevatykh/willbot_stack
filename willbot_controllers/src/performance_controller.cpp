#include <willbot_controllers/performance_controller.h>


namespace willbot_controllers
{
  bool PerformanceController::init(hardware_interface::JointStateInterface* /*hw*/, ros::NodeHandle & root_nh, ros::NodeHandle& controller_nh)
  {
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Duration>(root_nh, "control_step", 10));
    return true;
  }

  void PerformanceController::update(const ros::Time& /*time*/, const ros::Duration& period)
  {
      if (realtime_pub_->trylock())
      {
        realtime_pub_->msg_.data = period;
        realtime_pub_->unlockAndPublish();
      }
  }
} // namespace willbot_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::PerformanceController, controller_interface::ControllerBase)
