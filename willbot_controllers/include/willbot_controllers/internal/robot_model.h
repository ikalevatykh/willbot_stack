#pragma once

#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/node_handle.h>
#include <urdf/model.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <string>
#include <vector>

using namespace joint_limits_interface;

namespace willbot_controllers
{
class RobotModel
{
public:
  RobotModel()
  {
  }

  const std::string& root() const
  {
    return root_name_;
  }

  const std::string& tip() const
  {
    return tip_name_;
  }

  const std::vector<std::string>& joint_names() const
  {
    return joint_names_;
  }

  const KDL::Chain& chain() const
  {
    return kdl_chain_;
  }

  const JointLimits& joint_limits(const std::string& name) const
  {
    return joint_limits_.at(name);
  }

  const SoftJointLimits& soft_limits(const std::string& name) const
  {
    return soft_limits_.at(name);
  }

  bool init(ros::NodeHandle& nh);

private:
  std::string root_name_;
  std::string tip_name_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  std::vector<std::string> joint_names_;
  std::map<std::string, JointLimits> joint_limits_;
  std::map<std::string, SoftJointLimits> soft_limits_;
};

bool RobotModel::init(ros::NodeHandle& nh)
{
  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to parse robot description ('/robot_description')");
    return false;
  }

  if (!nh.getParam("root_name", root_name_))
  {
    ROS_ERROR_STREAM("No root name found on parameter server ('" << nh.getNamespace() << "/root_name')");
    return false;
  }

  if (!nh.getParam("tip_name", tip_name_))
  {
    ROS_ERROR_STREAM("No tip name found on parameter server ('" << nh.getNamespace() << "/root_name')");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
  {
    ROS_ERROR("Failed to construct KDL tree");
    return false;
  }

  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR_STREAM("Failed to get KDL chain: " << root_name_ << " -> " << tip_name_);
    return false;
  }

  // iterate thru all segments to find non-static joints
  for (const auto& segment : kdl_chain_.segments)
  {
    const auto& joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      const auto& name = joint.getName();
      const auto& urdf_joint = model.getJoint(name);

      if (!getJointLimits(name, nh, joint_limits_[name]))
        getJointLimits(urdf_joint, joint_limits_[name]);

      getSoftJointLimits(urdf_joint, soft_limits_[name]);

      joint_names_.push_back(name);
    }
  }

  return true;
}
}