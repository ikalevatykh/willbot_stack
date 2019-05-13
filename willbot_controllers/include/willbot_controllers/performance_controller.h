#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Duration.h>

#include <memory>

namespace willbot_controllers
{

class PerformanceController : public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
  PerformanceController() {}

  virtual bool init(hardware_interface::JointStateInterface * /*hw*/, ros::NodeHandle & /*root_nh*/, ros::NodeHandle &controller_nh);
  virtual void update(const ros::Time & /*time*/, const ros::Duration &period);

private:
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Duration>> realtime_pub_;
};

} // namespace willbot_controllers