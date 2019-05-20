#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <kdl/frames.hpp>

namespace willbot_controllers
{
class ForceTorqueSensorController : public controller_interface::Controller<hardware_interface::ForceTorqueSensorInterface>
{
public:
  virtual bool init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);

  virtual void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

protected:
  std::string name_;
  std::string frame_id_;
  std::string topic_;

  KDL::Wrench wrench_;

  // Wrench subscriber
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<KDL::Wrench> buffer_;
  void wrenchCB(const geometry_msgs::WrenchStampedConstPtr& msg);
};
}