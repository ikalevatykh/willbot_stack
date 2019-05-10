#pragma once

#include <geometry_msgs/WrenchStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/stiffness.hpp>

#include <memory>

namespace willbot_controllers
{
class Admittance
{
public:
  Admittance()
  {
  }

  bool init(ros::NodeHandle& nh, const KDL::Chain& chain, const std::string& frame_id);

  void reset(const KDL::JntArray& q_goal);

  void update(const KDL::JntArray& q_goal, const KDL::JntArray& q_curr, const KDL::Wrench& w_goal,
              const KDL::Wrench& w_curr, double dt);

private:
  ros::NodeHandle nh_;

  std::unique_ptr<KDL::ChainFkSolverPos> fk_;

  KDL::Frame compliance_frame_;
  KDL::Stiffness k_gain_;
  KDL::Stiffness d_gain_;

  KDL::JntArrayVel q_cmd_;
};

bool Admittance::init(ros::NodeHandle& nh, const KDL::Chain& chain, const std::string& frame_id)
{
  fk_.reset(new KDL::ChainFkSolverPos_recursive(chain));

  k_gain_ = KDL::Stiffness(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
  d_gain_ = KDL::Stiffness(1.0, 1e10, 1e10, 1e10, 1e10, 1e10);

  compliance_frame_ = KDL::Frame::Identity();

  return true;
}

void Admittance::reset(const KDL::JntArray& q_goal)
{
  q_cmd_ = KDL::JntArrayVel(q_goal);
}

void Admittance::update(const KDL::JntArray& q_goal, const KDL::JntArray& q_curr, const KDL::Wrench& w_goal,
                        const KDL::Wrench& w_curr, double dt)
{
  ROS_INFO_STREAM_THROTTLE(1, "dt = " << dt);  

  KDL::Frame x_goal;
  fk_->JntToCart(q_goal, x_goal);

  KDL::Frame x_curr;
  fk_->JntToCart(q_curr, x_curr);


  // D * (dx* - dx) + K * (x* - x) = F - F*

  // const auto& w_curr = compliance_frame_ * w_curr;

  const KDL::Twist& x_error = x_curr.M * diff(x_goal, x_curr, dt);
  const KDL::Wrench& w_error = w_goal - w_curr;

  const KDL::Twist& twist = d_gain_.Inverse(w_error - k_gain_ * x_error);

  ROS_INFO_STREAM_THROTTLE(1, "twist = " << twist[0] << ", " << twist[1] << ", " << twist[2] << ", " << twist[3] << ", "
                                         << twist[4] << ", " << twist[5]);
}
}