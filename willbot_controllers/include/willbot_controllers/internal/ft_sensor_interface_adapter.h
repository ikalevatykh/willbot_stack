#pragma once

#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>

namespace willbot_controllers
{
class ForceTorqueSensorInterface
{
public:
  ForceTorqueSensorInterface()
  {
  }

  inline hardware_interface::ForceTorqueSensorHandle getHandle() const
  {
    return hardware_interface::ForceTorqueSensorHandle(name_, frame_id_, force_, torque_);
  }

  bool init(ros::NodeHandle& nh);
  void starting();
  void update(const ros::Time& time);
  void stopping();

protected:
  void sensorCB(const geometry_msgs::WrenchStampedConstPtr& msg);

private:
  ros::NodeHandle nh_;

  std::string name_;
  std::string topic_;
  std::string frame_id_;
  double force_[3];
  double torque_[3];

  ros::Subscriber sub_sensor_;
  realtime_tools::RealtimeBuffer<geometry_msgs::WrenchStamped> msg_buffer_;
};

bool ForceTorqueSensorInterface::init(ros::NodeHandle& nh)
{
  if (!nh.getParam("force_torque_sensor/frame_id", frame_id_))
  {
    ROS_ERROR_STREAM("No frame_id found on parameter server ('" << nh.getNamespace() << "/frame_id')");
    return false;
  }

  if (!nh.getParam("force_torque_sensor/wrench_topic", topic_))
  {
    ROS_ERROR_STREAM("No wrench_topic found on parameter server ('" << nh.getNamespace() << "/wrench_topic')");
    return false;
  }

  nh_ = nh;
  return true;
}

void ForceTorqueSensorInterface::starting()
{
  auto hints = ros::TransportHints().unreliable();
  sub_sensor_ = nh_.subscribe(topic_, 1, &ForceTorqueSensorInterface::sensorCB, this, hints);

  force_[0] = force_[1] = force_[2] = 0.0;
  torque_[0] = torque_[1] = torque_[2] = 0.0;
}

void ForceTorqueSensorInterface::sensorCB(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  if (msg->header.frame_id != frame_id_)
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Force torque frame_id wrong, check '" << frame_id_ << "' topic.");
    return;
  }

  msg_buffer_.writeFromNonRT(*msg);
}

void ForceTorqueSensorInterface::update(const ros::Time& time)
{
  const auto& msg = *msg_buffer_.readFromRT();
  if (time - msg.header.stamp < ros::Duration(1, 0))
  {
    force_[0] = msg.wrench.force.x;
    force_[1] = msg.wrench.force.y;
    force_[2] = msg.wrench.force.z;

    torque_[0] = msg.wrench.torque.x;
    torque_[1] = msg.wrench.torque.y;
    torque_[2] = msg.wrench.torque.z;
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1, "Force torque data too old, check '" << topic_ << "' topic.");
  }
}

void ForceTorqueSensorInterface::stopping()
{
  sub_sensor_.shutdown();
}
}