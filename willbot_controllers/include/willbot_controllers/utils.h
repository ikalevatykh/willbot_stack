#pragma once

#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <string>
#include <vector>

namespace willbot_controllers
{
namespace utils
{
inline void getWrench(const hardware_interface::ForceTorqueSensorHandle& handle, KDL::Wrench& wrench)
{
  const auto* force = handle.getForce();
  wrench.force(0) = force[0];
  wrench.force(1) = force[1];
  wrench.force(2) = force[2];

  const auto* torque = handle.getTorque();
  wrench.torque(0) = torque[0];
  wrench.torque(1) = torque[1];
  wrench.torque(2) = torque[2];
}

inline void getPosition(const std::vector<hardware_interface::JointHandle>& handles, KDL::JntArray& q)
{
  for (int i = 0; i < 6; ++i)
  {
    q(i) = handles[i].getPosition();
  }
}

inline void setCommand(std::vector<hardware_interface::JointHandle>& handles, const KDL::JntArray& cmd)
{
  for (int i = 0; i < 6; ++i)
  {
    handles[i].setCommand(cmd(i));
  }
}

inline void zeroCommand(std::vector<hardware_interface::JointHandle>& handles)
{
  for (int i = 0; i < 6; ++i)
  {
    handles[i].setCommand(0.0);
  }
}

inline std::vector<std::string> getJointNames(const KDL::Chain& chain)
{
  std::vector<std::string> joint_names;
  for (const auto& segment : chain.segments)
  {
    const auto& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      joint_names.push_back(joint.getName());
    }
  }
  return joint_names;
}

}  // namespace utils
}  // namespace willbot_controllers