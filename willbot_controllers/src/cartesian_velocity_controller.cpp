#include <willbot_controllers/cartesian_velocity_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace willbot_controllers
{
bool CartesianVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                                       ros::NodeHandle& controller_nh)
{
  ros::NodeHandle& nh = controller_nh;
  urdf::Model model;
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;

  // Period for keeping previous command
  if (!nh.getParam("keep_command_duration", keep_command_duration_))
  {
    ROS_ERROR_STREAM("Could not find 'keep_command_duration' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Chain base frame
  if (!nh.getParam("chain/base_link", base_frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'chain/base_link' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Frame in which welocity will be applied
  if (!nh.getParam("chain/tool_link", tool_frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'chain/tool_link' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Robot kinematic model
  if (!model.initParam("/robot_description") || !kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to parse robot description ('/robot_description')");
    return false;
  }

  if (!kdl_tree.getChain(base_frame_id_, tool_frame_id_, kdl_chain))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain: " << base_frame_id_ << " -> " << tool_frame_id_);
    return false;
  }

  // Joint handles
  for (const auto& segment : kdl_chain.segments)
  {
    const auto& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      try
      {
        joint_handles_.push_back(hw->getHandle(joint.getName()));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }
  }
  n_joints_ = joint_handles_.size();

  // Prepare data
  ik_vel_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain));
  desired_ = KDL::JntArrayVel(n_joints_);

  sub_command_ = nh.subscribe<geometry_msgs::TwistStamped>("command", 1, &CartesianVelocityController::commandCB, this);
  return true;
}

void CartesianVelocityController::starting(const ros::Time& time)
{
  const auto command = command_buffer_.readFromRT();
  command->expired = ros::Time();
  command->twist = KDL::Twist::Zero();
}

void CartesianVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  const auto command = command_buffer_.readFromRT();
  const bool expired = time >= command->expired;
  const auto& twist = command->twist;

  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    desired_.q(i) = joint_handles_[i].getPosition();
  }

  if (expired || ik_vel_->CartToJnt(desired_.q, twist, desired_.qdot) != 0)
  {
    KDL::SetToZero(desired_.qdot);
  }

  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(desired_.qdot(i));
  }
}

void CartesianVelocityController::commandCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  if (msg->header.frame_id != base_frame_id_)
  {
    ROS_ERROR_STREAM("Frame id (" << msg->header.frame_id << ") does not match base frame id (" << base_frame_id_
                                  << ")! Not executing!");
    return;
  }

  const auto expired = ros::Time::now() + ros::Duration(keep_command_duration_);
  KDL::Twist twist;
  tf::twistMsgToKDL(msg->twist, twist);
  command_buffer_.writeFromNonRT({ expired, twist });
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::CartesianVelocityController, controller_interface::ControllerBase)
