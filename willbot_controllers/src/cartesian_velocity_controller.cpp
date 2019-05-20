#include <willbot_controllers/cartesian_velocity_controller.h>
#include <willbot_controllers/utils.h>

#include <kdl_conversions/kdl_msg.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace willbot_controllers
{
bool CartesianVelocityController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controller_nh)
{
  auto& nh = controller_nh;

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
  urdf::Model model;
  KDL::Tree tree;
  if (!model.initParam("/robot_description") || !kdl_parser::treeFromUrdfModel(model, tree))
  {
    ROS_ERROR("Failed to parse robot description ('/robot_description')");
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(base_frame_id_, tool_frame_id_, chain))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain: " << base_frame_id_ << " -> " << tool_frame_id_);
    return false;
  }

  try
  {
    // Joint handles
    for (const auto& joint_name : utils::getJointNames(chain))
    {
      joint_handles_.push_back(hw->getHandle(joint_name));
    }
    n_joints_ = joint_handles_.size();
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Exception thrown: " << e.what());
    return false;
  }

  // Prepare data
  ik_vel_.reset(new KDL::ChainIkSolverVel_wdls(chain));
  current_q_.resize(n_joints_);
  desired_qdot_.resize(n_joints_);
  previous_qdot_.resize(n_joints_);

  sub_command_ = nh.subscribe<geometry_msgs::TwistStamped>("command", 1, &CartesianVelocityController::commandCB, this);
  return true;
}

void CartesianVelocityController::starting(const ros::Time& time)
{
  const auto command = command_buffer_.readFromRT();
  command->expired = ros::Time();
  command->twist = KDL::Twist::Zero();

  KDL::SetToZero(previous_qdot_);
}

void CartesianVelocityController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  const auto command = command_buffer_.readFromRT();
  const bool expired = time >= command->expired;
  const auto& twist = command->twist;

  utils::getPosition(joint_handles_, current_q_);

  if (expired || ik_vel_->CartToJnt(current_q_, twist, desired_qdot_) != 0)
  {
    KDL::SetToZero(desired_qdot_);
  }

  utils::setCommand(joint_handles_, desired_qdot_);
}

void CartesianVelocityController::commandCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
  if (msg->header.frame_id != base_frame_id_)
  {
    ROS_ERROR_STREAM("Frame id (" << msg->header.frame_id << ") does not match base frame id (" << base_frame_id_
                                  << ")! Not executing!");
    return;
  }

  const auto expired =
      msg->header.stamp != ros::Time() ? msg->header.stamp : ros::Time::now() + ros::Duration(keep_command_duration_);

  KDL::Twist twist;
  tf::twistMsgToKDL(msg->twist, twist);
  command_buffer_.writeFromNonRT({ expired, twist });
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::CartesianVelocityController, controller_interface::ControllerBase)
