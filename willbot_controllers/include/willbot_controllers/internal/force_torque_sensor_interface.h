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

  inline hardware_interface::ForceTorqueSensorHandle getHandle()
  {
    return hardware_interface::ForceTorqueSensorHandle(name_, frame_id_, force_, torque_);
  }

  bool init(ros::NodeHandle& nh);
  void update(const ros::Time& time);

protected:
  void sensorCB(const geometry_msgs::WrenchStampedConstPtr& msg);

private:
  std::string name_;
  std::string topic_;
  std::string frame_id_;

  ros::Subscriber sub_sensor_;
  realtime_tools::RealtimeBuffer<geometry_msgs::WrenchStamped> msg_buffer_;

  double force_[3];
  double torque_[3];
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

  // subscribe to wrench topic
  auto hints = ros::TransportHints().unreliable();
  sub_sensor_ = nh.subscribe(topic_, 1, &ForceTorqueSensorInterface::sensorCB, this, hints);

  // waiting for message
  ros::Duration timeout = ros::Duration(1.0);
  ros::Time start = ros::Time::now();
  while (nh.ok())
  {
    ros::WallDuration(0.1).sleep();

    const auto& msg = *msg_buffer_.readFromRT();
    if (msg.header.stamp > start)
      break;

    if (ros::Time::now() >= start + timeout)
    {
      ROS_ERROR_STREAM("Timeout exceeded while waiting for message on topic " << topic_);
      return false;
    }
  }

  return true;
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
    ROS_WARN_STREAM_THROTTLE(1, "Message on topic " << topic_ << " too old.");
  }
}

void ForceTorqueSensorInterface::sensorCB(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  if (msg->header.frame_id != frame_id_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Message on topic " << topic_ << " has different frame_id.");
    return;
  }

  msg_buffer_.writeFromNonRT(*msg);
}
}