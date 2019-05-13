#pragma once

#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>


namespace willbot_controllers
{
    inline void get_wrench(const hardware_interface::ForceTorqueSensorHandle& handle, KDL::Wrench& wrench)
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

    inline void get_position(const std::vector<hardware_interface::JointHandle>& handles, KDL::JntArray& q)
    {
        for (unsigned int i = 0; i < handles.size(); ++i)
        {
            q(i) = handles[i].getPosition();
        }
    }

    inline void set_command(const KDL::JntArray& cmd, std::vector<hardware_interface::JointHandle>& handles)
    {
        for (unsigned int i = 0; i < handles.size(); ++i)
        {
            handles[i].setCommand(cmd(i));
        }
    }

} // namespace willbot_controllers