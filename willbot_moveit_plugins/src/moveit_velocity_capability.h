#ifndef MOVEIT_MOVE_GROUP_VELOCITY_CAPABILITY_
#define MOVEIT_MOVE_GROUP_VELOCITY_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>

namespace move_group
{
class MoveGroupVelocityControl : public MoveGroupCapability
{
  public:
    MoveGroupVelocityControl();

    void initialize() override;

  private:
};

} // namespace move_group

#endif // MOVEIT_MOVE_GROUP_VELOCITY_CAPABILITY_