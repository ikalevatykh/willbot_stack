#include "moveit_velocity_capability.h"

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{

MoveGroupVelocityControl::MoveGroupVelocityControl() 
    : MoveGroupCapability("VelocityControl")
{
}

void MoveGroupVelocityControl::initialize()
{
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupVelocityControl, move_group::MoveGroupCapability)