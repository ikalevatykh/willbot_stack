#include <willbot_controllers/cartesian_admittance_controller.h>
#include <willbot_controllers/utils.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace willbot_controllers
{
bool CartesianAdmittanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh)
{
  auto* joints_hw = robot_hw->get<hardware_interface::VelocityJointInterface>();
  auto* sensor_hw = robot_hw->get<hardware_interface::ForceTorqueSensorInterface>();

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

  // Force torque sensor name
  if (!nh.getParam("force_torque_sensor", sensor_name_))
  {
    ROS_ERROR_STREAM("Could not find 'force_torque_sensor' prameter (namespace: " << nh.getNamespace() << ").");
    return false;
  }

  // Robot kinematic model
  urdf::Model model;
  KDL::Tree kdl_tree;
  if (!model.initParam("/robot_description") || !kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    ROS_ERROR("Failed to parse robot description ('/robot_description')");
    return false;
  }

  KDL::Chain chain;
  if (!kdl_tree.getChain(base_frame_id_, tool_frame_id_, chain))
  {
    ROS_ERROR_STREAM("Failed to get chain: " << base_frame_id_ << " -> " << tool_frame_id_);
    return false;
  }

  try
  {
    // Joint handles
    for (const auto& joint_name : utils::getJointNames(chain))
      joint_handles_.push_back(joints_hw->getHandle(joint_name));

    // Sensor handle
    sensor_handle_ = sensor_hw->getHandle(sensor_name_);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Exception thrown: " << e.what());
    return false;
  }

  // Transformation between ft sensor and tool frame
  KDL::Chain fts_chain;
  if (!kdl_tree.getChain(tool_frame_id_, sensor_handle_.getFrameId(), fts_chain))
  {
    ROS_ERROR_STREAM("Failed to get chain: " << sensor_handle_.getFrameId() << " -> " << tool_frame_id_);
    return false;
  }
  KDL::ChainFkSolverPos_recursive fk_pos{ fts_chain };
  fk_pos.JntToCart(KDL::JntArray{}, fts_to_tool_);

  // Kinematic solvers
  fk_pos_.reset(new KDL::ChainFkSolverPos_recursive{chain});
  ik_vel_.reset(new KDL::ChainIkSolverVel_wdls{chain});

  // Prepare data
  n_joints_ = joint_handles_.size();
  q_equilibrium_.resize(n_joints_);
  q_current_.resize(n_joints_);
  qdot_desired_.resize(n_joints_);
  axis_enabled_.assign(6, false);

  // Dynamic reconfigure
  dynamic_server_admittance_param_ =
      std::make_unique<dynamic_reconfigure::Server<willbot_controllers::admittance_paramConfig>>(nh);
  dynamic_server_admittance_param_->setCallback(
      boost::bind(&CartesianAdmittanceController::admittanceParamCallback, this, _1, _2));

  return true;
}

void CartesianAdmittanceController::starting(const ros::Time& time)
{
  // Equilibrium position
  utils::getPosition(joint_handles_, q_equilibrium_);
  fk_pos_->JntToCart(q_equilibrium_, x_equilibrium_);

  // Bias wrench
  utils::getWrench(sensor_handle_, wrench_bias_);

  // Start from zero velocity
  twist_desired_ = KDL::Twist::Zero();
}

void CartesianAdmittanceController::update(const ros::Time& /*time*/, const ros::Duration& period)
{
  const auto dt = period.toSec();

  // Current state
  utils::getWrench(sensor_handle_, wrench_current_);
  utils::getPosition(joint_handles_, q_current_);

  fk_pos_->JntToCart(q_current_, x_current_);

  // Wrench in sensor frame
  const auto wrench_sens = wrench_current_ - wrench_bias_;
  // Wrench in base axes
  const auto wrench_base = x_current_.M * (fts_to_tool_ * wrench_sens);
  // Error in base axes
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
  if (ik_vel_->CartToJnt(q_current_, twist_desired_, qdot_desired_) != 0)
  {
    SetToZero(qdot_desired_);
  }

  utils::setCommand(joint_handles_, qdot_desired_);
}

void CartesianAdmittanceController::admittanceParamCallback(willbot_controllers::admittance_paramConfig& config,
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
