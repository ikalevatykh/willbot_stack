#include <willbot_controllers/cartesian_admittance_controller.h>

#include <limits>
#include <memory>

#include <kdl_conversions/kdl_msg.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace willbot_controllers
{
bool CartesianAdmittanceController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                                         ros::NodeHandle& controller_nh)
{
  ros::NodeHandle& nh = controller_nh;
  urdf::Model model;
  KDL::Tree kdl_tree;
  KDL::Chain chain;
  KDL::Chain fts_chain;

  // Chain base frame
  if (!nh.getParam("chain/base_link", base_frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'chain/base_link' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Chain tool frame
  if (!nh.getParam("chain/tool_link", tool_frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'chain/tool_link' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Force torque sensor measure frame
  if (!nh.getParam("force_torque_sensor/frame", wrench_frame_id_))
  {
    ROS_ERROR_STREAM("Could not find 'force_torque_sensor/frame' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Force torque sensor topic
  if (!nh.getParam("force_torque_sensor/topic", wrench_topic_))
  {
    ROS_ERROR_STREAM("Could not find 'force_torque_sensor/topic' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Robot kinematic model
  if (!model.initParam("/robot_description") || !kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to parse robot description ('/robot_description')");
    return false;
  }

  if (!kdl_tree.getChain(base_frame_id_, tool_frame_id_, chain))
  {
    ROS_ERROR_STREAM("Failed to get chain: " << base_frame_id_ << " -> " << tool_frame_id_);
    return false;
  }

  if (!kdl_tree.getChain(tool_frame_id_, wrench_frame_id_, fts_chain))
  {
    ROS_ERROR_STREAM("Failed to get chain: " << wrench_frame_id_ << " -> " << tool_frame_id_);
    return false;
  }

  // Joint handles
  for (const auto& segment : chain.segments)
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

  axis_enabled_.assign(6, false);

  // Dynamic reconfigure
  dynamic_server_compliance_param_ =
      std::make_unique<dynamic_reconfigure::Server<willbot_controllers::compliance_paramConfig>>(nh);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianAdmittanceController::complianceParamCallback, this, _1, _2));

  // Force torque data subscriber
  sub_wrench_ =
      nh.subscribe<geometry_msgs::WrenchStamped>(wrench_topic_, 100, &CartesianAdmittanceController::wrenchCB, this);

  // Kinematic solvers
  fk_pos_.reset(new KDL::ChainFkSolverPos_recursive(chain));
  ik_vel_.reset(new KDL::ChainIkSolverVel_wdls(chain));

  // Transformation between ft sensor and tool frame
  KDL::ChainFkSolverPos_recursive fk_pos{ fts_chain };
  fk_pos.JntToCart(KDL::JntArray{}, fts_to_tool_);

  // Prepare data
  q_equilibrium_.resize(n_joints_);
  q_current_.resize(n_joints_);
  qdot_desired_.resize(n_joints_);

  return true;
}

void CartesianAdmittanceController::starting(const ros::Time& time)
{
  // Equilibrium position
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    q_equilibrium_(i) = joint_handles_[i].getPosition();
  }
  fk_pos_->JntToCart(q_equilibrium_, x_equilibrium_);

  // Start from zero velocity
  twist_desired_ = KDL::Twist::Zero();
  // Wait for first wrench to setup bias
  wrench_bias_ok_ = false;
}

void CartesianAdmittanceController::update(const ros::Time& time, const ros::Duration& period)
{
  const auto dt = period.toSec();

  // Bias is important
  if (!wrench_bias_ok_)
  {
    ROS_WARN_STREAM_DELAYED_THROTTLE(1, "Waiting for force torque data");
    wrench_bias_ = *wrench_buffer_.readFromRT();
    wrench_bias_ok_ = wrench_bias_ != KDL::Wrench::Zero();
    return;
  }

  // Current position
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    q_current_(i) = joint_handles_[i].getPosition();
  }
  fk_pos_->JntToCart(q_current_, x_current_);

  // Wrench in sensor frame
  const auto wrench_sens = *wrench_buffer_.readFromRT() - wrench_bias_;
  // Wrench in base frame
  const auto wrench_base = x_current_.M * (fts_to_tool_ * wrench_sens);
  // Error in base frame
  const auto error_base = KDL::diff(x_equilibrium_, x_current_);

  // Admittance dynamics: M*x" + D*x' + K*e = F
  for (int i = 0; i < 6; ++i)
  {
    if (axis_enabled_[i])
    {
      const auto accel = (wrench_base[i] - k_gain_[i] * error_base[i] - d_gain_[i] * twist_desired_[i]) / m_gain_[i];
      twist_desired_[i] += accel * dt;
    }
    else
    {
      twist_desired_[i] = 0.0;
    }
  }

  // Desired joint velocities
  ik_vel_->CartToJnt(q_current_, twist_desired_, qdot_desired_);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    joint_handles_[i].setCommand(qdot_desired_(i));
  }
}

void CartesianAdmittanceController::wrenchCB(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  if (msg->header.frame_id != wrench_frame_id_)
  {
    ROS_ERROR_STREAM("Frame id (" << msg->header.frame_id << ") does not match sensor frame id (" << wrench_frame_id_
                                  << ")! Not executing!");
    return;
  }

  tf::wrenchMsgToKDL(msg->wrench, wrench_current_);
  wrench_buffer_.writeFromNonRT(wrench_current_);
}

void CartesianAdmittanceController::complianceParamCallback(willbot_controllers::compliance_paramConfig& config,
                                                            uint32_t /*level*/)
{
  axis_enabled_ = { config.x_enabled,  config.y_enabled,  config.z_enabled,
                    config.rx_enabled, config.ry_enabled, config.rz_enabled };
  k_gain_ = KDL::Stiffness{ config.x_K, config.y_K, config.z_K, config.rx_K, config.ry_K, config.rz_K };
  d_gain_ = KDL::Stiffness{ config.x_D, config.y_D, config.z_D, config.rx_D, config.ry_D, config.rz_D };
  m_gain_ = KDL::Stiffness{ config.x_M, config.y_M, config.z_M, config.rx_M, config.ry_M, config.rz_M };
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(willbot_controllers::CartesianAdmittanceController, controller_interface::ControllerBase)
