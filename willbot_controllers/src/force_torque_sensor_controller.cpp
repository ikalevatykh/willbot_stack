#include <willbot_controllers/force_torque_sensor_controller.h>

#include <kdl_conversions/kdl_msg.h>

namespace willbot_controllers
{
bool ForceTorqueSensorController::init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle& root_nh,
                                       ros::NodeHandle& controller_nh)
{
  auto& nh = controller_nh;

  // Force torque sensor id
  if (!nh.getParam("sensor/name", name_))
  {
    ROS_ERROR_STREAM("Could not find 'sensor/name' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Force torque sensor frame
  if (!nh.getParam("sensor/frame_id", frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'sensor/frame_id' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Force torque sensor topic
  if (!nh.getParam("sensor/topic", topic_))
  {
    ROS_ERROR_STREAM("Could not find 'sensor/topic' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Register the sensor handle if it is not registered yet
  // [This is a hack that we use because cannot register ft hardware with ur_modern_driver]
  try
  {
    hw->getHandle(name_);

    ROS_ERROR_STREAM("ForceTorqueSensorHandle " << name_ << " already registered.");
    return false;
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    hw->registerHandle(
        hardware_interface::ForceTorqueSensorHandle(name_, frame_id_, wrench_.force.data, wrench_.torque.data));
  }

  // Force torque data subscriber
  sub_ = nh.subscribe<geometry_msgs::WrenchStamped>(topic_, 10, &ForceTorqueSensorController::wrenchCB, this);
  return true;
}

void ForceTorqueSensorController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  wrench_ = *buffer_.readFromRT();
}

void ForceTorqueSensorController::wrenchCB(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  if (msg->header.frame_id != frame_id_)
  {
    ROS_ERROR_STREAM("Frame id (" << msg->header.frame_id << ") does not match sensor frame id (" << frame_id_
                                  << ")! Not executing!");
    return;
  }

  KDL::Wrench wrench_kdl;
  tf::wrenchMsgToKDL(msg->wrench, wrench_kdl);
  buffer_.writeFromNonRT(wrench_kdl);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::ForceTorqueSensorController, controller_interface::ControllerBase)
