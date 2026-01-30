#include "nav2_mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_mpc_controller
{

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MpcController: failed to lock node");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  using nav2_util::declare_parameter_if_not_declared;

  // ---- Declare params (namespaced by plugin name) ----
  declare_parameter_if_not_declared(node, plugin_name_ + ".N", rclcpp::ParameterValue(p_.N));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dt", rclcpp::ParameterValue(p_.dt));

  declare_parameter_if_not_declared(node, plugin_name_ + ".v_max", rclcpp::ParameterValue(p_.v_max));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_min", rclcpp::ParameterValue(p_.v_min));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(p_.w_max));

  declare_parameter_if_not_declared(node, plugin_name_ + ".dv_max", rclcpp::ParameterValue(p_.dv_max));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dw_max", rclcpp::ParameterValue(p_.dw_max));

  // Costs
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_pos", rclcpp::ParameterValue(p_.w_pos));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_yaw", rclcpp::ParameterValue(p_.w_yaw));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_v", rclcpp::ParameterValue(p_.w_v));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_w", rclcpp::ParameterValue(p_.w_w));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_dv", rclcpp::ParameterValue(p_.w_dv));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_dw", rclcpp::ParameterValue(p_.w_dw));

  // Terminal costs
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_terminal_pos", rclcpp::ParameterValue(p_.w_terminal_pos));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_terminal_yaw", rclcpp::ParameterValue(p_.w_terminal_yaw));

  // Progress + stop gates
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".w_progress", rclcpp::ParameterValue(p_.w_progress));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".v_ref", rclcpp::ParameterValue(p_.v_ref));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".progress_stop_dist", rclcpp::ParameterValue(p_.progress_stop_dist));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".yaw_stop_dist", rclcpp::ParameterValue(p_.yaw_stop_dist));

  // Costmap / collision
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_costmap", rclcpp::ParameterValue(p_.w_costmap));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lethal_cost", rclcpp::ParameterValue(p_.lethal_cost));

  // Optimizer
  declare_parameter_if_not_declared(node, plugin_name_ + ".opt_iters", rclcpp::ParameterValue(p_.opt_iters));
  declare_parameter_if_not_declared(node, plugin_name_ + ".step_size", rclcpp::ParameterValue(p_.step_size));
  declare_parameter_if_not_declared(node, plugin_name_ + ".grad_eps", rclcpp::ParameterValue(p_.grad_eps));

  // Path handling
  declare_parameter_if_not_declared(node, plugin_name_ + ".prune_dist", rclcpp::ParameterValue(p_.prune_dist));
  declare_parameter_if_not_declared(node, plugin_name_ + ".ref_stride", rclcpp::ParameterValue(p_.ref_stride));
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(p_.transform_tolerance));

  // Output smoothing
  declare_parameter_if_not_declared(node, plugin_name_ + ".cmd_smoothing", rclcpp::ParameterValue(p_.cmd_smoothing));

  // Debug
  declare_parameter_if_not_declared(node, plugin_name_ + ".publish_debug", rclcpp::ParameterValue(p_.publish_debug));

  // ---- Get params ----
  node->get_parameter(plugin_name_ + ".N", p_.N);
  node->get_parameter(plugin_name_ + ".dt", p_.dt);

  node->get_parameter(plugin_name_ + ".v_max", p_.v_max);
  node->get_parameter(plugin_name_ + ".v_min", p_.v_min);
  node->get_parameter(plugin_name_ + ".w_max", p_.w_max);

  node->get_parameter(plugin_name_ + ".dv_max", p_.dv_max);
  node->get_parameter(plugin_name_ + ".dw_max", p_.dw_max);

  node->get_parameter(plugin_name_ + ".w_pos", p_.w_pos);
  node->get_parameter(plugin_name_ + ".w_yaw", p_.w_yaw);
  node->get_parameter(plugin_name_ + ".w_v", p_.w_v);
  node->get_parameter(plugin_name_ + ".w_w", p_.w_w);
  node->get_parameter(plugin_name_ + ".w_dv", p_.w_dv);
  node->get_parameter(plugin_name_ + ".w_dw", p_.w_dw);

  node->get_parameter(plugin_name_ + ".w_terminal_pos", p_.w_terminal_pos);
  node->get_parameter(plugin_name_ + ".w_terminal_yaw", p_.w_terminal_yaw);

  node->get_parameter(plugin_name_ + ".w_progress", p_.w_progress);
  node->get_parameter(plugin_name_ + ".v_ref", p_.v_ref);
  node->get_parameter(plugin_name_ + ".progress_stop_dist", p_.progress_stop_dist);
  node->get_parameter(plugin_name_ + ".yaw_stop_dist", p_.yaw_stop_dist);

  node->get_parameter(plugin_name_ + ".w_costmap", p_.w_costmap);
  node->get_parameter(plugin_name_ + ".lethal_cost", p_.lethal_cost);

  node->get_parameter(plugin_name_ + ".opt_iters", p_.opt_iters);
  node->get_parameter(plugin_name_ + ".step_size", p_.step_size);
  node->get_parameter(plugin_name_ + ".grad_eps", p_.grad_eps);

  node->get_parameter(plugin_name_ + ".prune_dist", p_.prune_dist);
  node->get_parameter(plugin_name_ + ".ref_stride", p_.ref_stride);
  node->get_parameter(plugin_name_ + ".transform_tolerance", p_.transform_tolerance);

  node->get_parameter(plugin_name_ + ".cmd_smoothing", p_.cmd_smoothing);
  node->get_parameter(plugin_name_ + ".publish_debug", p_.publish_debug);

  if (p_.N < 1) {
    throw std::runtime_error("MpcController: N must be >= 1");
  }
  if (p_.dt <= 0.0) {
    throw std::runtime_error("MpcController: dt must be > 0");
  }

  // Warm start / state
  u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  last_u_.setZero();
  v_max_eff_ = p_.v_max;
  has_speed_limit_ = false;

  // Collision checker
  collision_checker_ =
    std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
      costmap_ros_->getCostmap());

  // Debug publishers (lifecycle pubs)
  pub_local_plan_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/local_plan", rclcpp::SystemDefaultsQoS());
  pub_pred_path_ = node->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/predicted_path", rclcpp::SystemDefaultsQoS());

  // Dynamic parameter callback
  param_cb_ = node->add_on_set_parameters_callback(
    std::bind(&MpcController::onParamChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "Configured MPC controller '%s' (N=%d dt=%.3f, ROS 2 Jazzy)",
    plugin_name_.c_str(), p_.N, p_.dt);
}

void MpcController::cleanup()
{
  has_plan_ = false;
  global_plan_.poses.clear();
  goal_pose_ = geometry_msgs::msg::PoseStamped();

  u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  last_u_.setZero();

  collision_checker_.reset();
  pub_local_plan_.reset();
  pub_pred_path_.reset();
  param_cb_.reset();
}

void MpcController::activate()
{
  if (pub_local_plan_) pub_local_plan_->on_activate();
  if (pub_pred_path_) pub_pred_path_->on_activate();
}

void MpcController::deactivate()
{
  if (pub_local_plan_) pub_local_plan_->on_deactivate();
  if (pub_pred_path_) pub_pred_path_->on_deactivate();
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  has_plan_ = !global_plan_.poses.empty();

  if (has_plan_) {
    goal_pose_ = global_plan_.poses.back();
  }

  // Reset warm start on new plan
  u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  last_u_.setZero();
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Convention: if speed_limit <= 0 => reset to configured max
  if (speed_limit <= 0.0) {
    has_speed_limit_ = false;
    v_max_eff_ = p_.v_max;
    return;
  }

  has_speed_limit_ = true;

  if (percentage) {
    const double pct = std::max(0.0, std::min(100.0, speed_limit)) / 100.0;
    v_max_eff_ = std::max(0.0, pct * p_.v_max);
  } else {
    v_max_eff_ = std::max(0.0, speed_limit);
  }
}

nav_msgs::msg::Path MpcController::transformPlanToBaseFrame(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  nav_msgs::msg::Path out;
  out.header.frame_id = costmap_ros_->getBaseFrameID();
  out.header.stamp = robot_pose.header.stamp;

  if (plan.poses.empty()) {
    return out;
  }

  const std::string base = costmap_ros_->getBaseFrameID();

  // Prune to p_.prune_dist to keep it small
  double acc_dist = 0.0;
  geometry_msgs::msg::PoseStamped prev = plan.poses.front();

  for (size_t i = 0; i < plan.poses.size(); ++i) {
    const auto & p_in = plan.poses[i];

    if (i > 0) {
      acc_dist += nav2_util::geometry_utils::euclidean_distance(prev, p_in);
      prev = p_in;
      if (acc_dist > p_.prune_dist) {
        break;
      }
    }

    geometry_msgs::msg::PoseStamped p_out;
    try {
      tf_->transform(p_in, p_out, base, tf2::durationFromSec(p_.transform_tolerance));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "TF transform plan->base failed: %s", ex.what());
      break;
    }

    out.poses.push_back(p_out);
  }

  return out;
}

void MpcController::buildReferenceHorizon(
  const nav_msgs::msg::Path & local_plan_base,
  int N,
  Eigen::MatrixXd & xref) const
{
  xref = Eigen::MatrixXd::Zero(N, 3);

  const int M = static_cast<int>(local_plan_base.poses.size());
  if (M == 0) {
    return;
  }

  int idx = 0;
  for (int k = 0; k < N; ++k) {
    idx = std::min(idx, M - 1);

    const auto & ps = local_plan_base.poses[idx];
    const double x = ps.pose.position.x;
    const double y = ps.pose.position.y;

    // --- yaw from path tangent (NOT pose orientation) ---
    double yaw = 0.0;
    if (M >= 2) {
      int i0 = idx;
      int i1 = std::min(idx + 1, M - 1);

      // If we're at the last point, use backward difference
      if (i0 == i1 && idx > 0) {
        i0 = idx - 1;
        i1 = idx;
      }

      const auto & p0 = local_plan_base.poses[i0].pose.position;
      const auto & p1 = local_plan_base.poses[i1].pose.position;
      yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    }

    xref(k, 0) = x;
    xref(k, 1) = y;
    xref(k, 2) = yaw;

    // unwrap yaw to be continuous across the horizon
    if (k > 0) {
      const double prev = xref(k - 1, 2);
      double cur = xref(k, 2);
      while (cur - prev > M_PI) cur -= 2.0 * M_PI;
      while (cur - prev < -M_PI) cur += 2.0 * M_PI;
      xref(k, 2) = cur;
    }

    idx += std::max(1, p_.ref_stride);
    if (idx >= M) {
      idx = M - 1;  // hold last
    }
  }
}

void MpcController::cacheCycleTransform_(const rclcpp::Time & stamp)
{
  (void)stamp;

  have_cycle_tf_ = false;

  geometry_msgs::msg::TransformStamped tf_gb;
  try {
    // global <- base
    tf_gb = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(),
      costmap_ros_->getBaseFrameID(),
      tf2::TimePointZero,  // latest available (robust)
      tf2::durationFromSec(p_.transform_tolerance));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "MPC: lookupTransform(global<-base) failed: %s", ex.what());
    return;
  }

  cycle_gx_ = tf_gb.transform.translation.x;
  cycle_gy_ = tf_gb.transform.translation.y;
  cycle_gyaw_ = tf2::getYaw(tf_gb.transform.rotation);
  cycle_c_ = std::cos(cycle_gyaw_);
  cycle_s_ = std::sin(cycle_gyaw_);
  have_cycle_tf_ = true;
}

Eigen::Vector3d MpcController::stepDiffDrive(
  const Eigen::Vector3d & x,
  const Eigen::Vector2d & u,
  double dt)
{
  const double px = x(0);
  const double py = x(1);
  const double yaw = x(2);

  const double v = u(0);
  const double w = u(1);

  Eigen::Vector3d xn;
  xn(0) = px + dt * v * std::cos(yaw);
  xn(1) = py + dt * v * std::sin(yaw);
  xn(2) = yaw + dt * w;
  return xn;
}

Eigen::Vector2d MpcController::clampControl(const Eigen::Vector2d & u) const
{
  Eigen::Vector2d uc = u;

  const double v_max = v_max_eff_;
  uc(0) = std::max(p_.v_min, std::min(v_max, uc(0)));
  uc(1) = std::max(-p_.w_max, std::min(p_.w_max, uc(1)));
  return uc;
}

Eigen::Vector2d MpcController::clampAccel(
  const Eigen::Vector2d & u,
  const Eigen::Vector2d & u_prev) const
{
  Eigen::Vector2d out = u;

  const double dv = out(0) - u_prev(0);
  const double dw = out(1) - u_prev(1);

  const double dv_lim = std::max(0.0, p_.dv_max) * p_.dt;
  const double dw_lim = std::max(0.0, p_.dw_max) * p_.dt;

  out(0) = u_prev(0) + std::max(-dv_lim, std::min(dv_lim, dv));
  out(1) = u_prev(1) + std::max(-dw_lim, std::min(dw_lim, dw));
  return out;
}

double MpcController::costmapCostAtBasePose(const Eigen::Vector3d & x_base) const
{
  if (!have_cycle_tf_) {
    return 255.0;
  }

  // Transform (x,y) in base frame into global frame using cached transform
  const double xb = x_base(0);
  const double yb = x_base(1);

  const double xg = cycle_gx_ + cycle_c_ * xb - cycle_s_ * yb;
  const double yg = cycle_gy_ + cycle_s_ * xb + cycle_c_ * yb;

  unsigned int mx, my;
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap->worldToMap(xg, yg, mx, my)) {
    return 255.0;  // outside map
  }

  return static_cast<double>(costmap->getCost(mx, my));
}

bool MpcController::inCollisionAtBasePose(const Eigen::Vector3d & x_base) const
{
  if (!have_cycle_tf_) {
    return true;
  }

  // Global pose from cached base->global transform
  const double xb = x_base(0);
  const double yb = x_base(1);
  const double yawb = x_base(2);

  const double xg = cycle_gx_ + cycle_c_ * xb - cycle_s_ * yb;
  const double yg = cycle_gy_ + cycle_s_ * xb + cycle_c_ * yb;
  const double yawg = cycle_gyaw_ + yawb;

  // Footprint collision in global frame
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double fp_cost = collision_checker_->footprintCostAtPose(xg, yg, yawg, footprint);

  if (fp_cost < 0.0) {
    return true;
  }

  // Also treat high cell costs as lethal-ish
  return costmapCostAtBasePose(x_base) >= p_.lethal_cost;
}

double MpcController::rolloutCost(
  const Eigen::Vector3d & x0,
  const Eigen::MatrixXd & u_seq,
  const Eigen::MatrixXd & xref,
  const Eigen::Vector2d & u_prev) const
{
  Eigen::Vector3d x = x0;
  Eigen::Vector2d u_last = u_prev;

  double J = 0.0;

  for (int k = 0; k < u_seq.rows(); ++k) {
    Eigen::Vector2d u(u_seq(k, 0), u_seq(k, 1));
    u = clampControl(u);
    u = clampAccel(u, u_last);

    // Collision / costmap penalty at current predicted pose
    if (inCollisionAtBasePose(x)) {
      return 1e9;
    }
    const double cell_cost = costmapCostAtBasePose(x);
    if (cell_cost >= p_.lethal_cost) {
      return 1e9;
    } else {
      const double c = cell_cost / std::max(1.0, p_.lethal_cost);
      J += p_.w_costmap * (c * c);
    }

    // Tracking error (base frame)
    const double ex = x(0) - xref(k, 0);
    const double ey = x(1) - xref(k, 1);
    const double dist_ref = std::hypot(ex, ey);

    // Progress encouragement (DISABLE near target)
    double wprog = p_.w_progress;
    if (dist_ref < p_.progress_stop_dist) {
      wprog = 0.0;
    }
    J += wprog * (p_.v_ref - u(0)) * (p_.v_ref - u(0));

    // Yaw error wrapped
    double epsi = x(2) - xref(k, 2);
    while (epsi > M_PI) epsi -= 2.0 * M_PI;
    while (epsi < -M_PI) epsi += 2.0 * M_PI;

    // Stop caring about path tangent yaw near the target
    double w_yaw_k = p_.w_yaw;
    if (dist_ref < p_.yaw_stop_dist) {
      w_yaw_k = 0.0;
    }

    // Input smoothness (approx accel)
    const double dv = (u(0) - u_last(0)) / p_.dt;
    const double dw = (u(1) - u_last(1)) / p_.dt;

    // Main running cost
    J += p_.w_pos * (ex * ex + ey * ey)
      + w_yaw_k * (epsi * epsi)
      + p_.w_v * (u(0) * u(0))
      + p_.w_w * (u(1) * u(1))
      + p_.w_dv * (dv * dv)
      + p_.w_dw * (dw * dw);

    // Propagate
    x = stepDiffDrive(x, u, p_.dt);
    u_last = u;
  }

  // Terminal cost to last reference point
  const int kN = static_cast<int>(xref.rows()) - 1;
  const double exN = x(0) - xref(kN, 0);
  const double eyN = x(1) - xref(kN, 1);

  double epsiN = x(2) - xref(kN, 2);
  while (epsiN > M_PI) epsiN -= 2.0 * M_PI;
  while (epsiN < -M_PI) epsiN += 2.0 * M_PI;

  J += p_.w_terminal_pos * (exN * exN + eyN * eyN)
    + p_.w_terminal_yaw * (epsiN * epsiN);

  // Also check terminal pose collision lightly
  if (inCollisionAtBasePose(x)) {
    return 1e9;
  }

  return J;
}

void MpcController::optimizeControlSequence(
  const Eigen::Vector3d & x0,
  const Eigen::MatrixXd & xref,
  const Eigen::Vector2d & u_prev,
  Eigen::MatrixXd & u_seq_io)
{
  const int N = static_cast<int>(u_seq_io.rows());
  const double eps = p_.grad_eps;

  for (int it = 0; it < p_.opt_iters; ++it) {
    const double J0 = rolloutCost(x0, u_seq_io, xref, u_prev);

    Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(N, 2);

    for (int k = 0; k < N; ++k) {
      for (int j = 0; j < 2; ++j) {
        Eigen::MatrixXd u_pert = u_seq_io;
        u_pert(k, j) += eps;
        const double Jp = rolloutCost(x0, u_pert, xref, u_prev);
        grad(k, j) = (Jp - J0) / eps;
      }
    }

    u_seq_io -= p_.step_size * grad;

    // Clamp each control after update (and accel clamp sequence-wise)
    Eigen::Vector2d u_last = u_prev;
    for (int k = 0; k < N; ++k) {
      Eigen::Vector2d u(u_seq_io(k, 0), u_seq_io(k, 1));
      u = clampControl(u);
      u = clampAccel(u, u_last);

      u_seq_io(k, 0) = u(0);
      u_seq_io(k, 1) = u(1);

      u_last = u;
    }
  }
}

nav_msgs::msg::Path MpcController::makePredictedPathMsg(
  const Eigen::Vector3d & x0,
  const Eigen::MatrixXd & u_seq,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getBaseFrameID();
  path.header.stamp = stamp;

  Eigen::Vector3d x = x0;

  for (int k = 0; k < u_seq.rows(); ++k) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;

    ps.pose.position.x = x(0);
    ps.pose.position.y = x(1);
    ps.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x(2));
    ps.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(ps);

    Eigen::Vector2d u(u_seq(k, 0), u_seq(k, 1));
    u = clampControl(u);
    x = stepDiffDrive(x, u, p_.dt);
  }

  // Add terminal pose
  geometry_msgs::msg::PoseStamped ps;
  ps.header = path.header;
  ps.pose.position.x = x(0);
  ps.pose.position.y = x(1);
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x(2));
  ps.pose.orientation = tf2::toMsg(q);
  path.poses.push_back(ps);

  return path;
}

rcl_interfaces::msg::SetParametersResult MpcController::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & p : params) {
    const auto & n = p.get_name();

    auto match = [&](const std::string & key) {
      return n == plugin_name_ + "." + key;
    };

    try {
      if (match("N")) {
        int N = p.as_int();
        if (N < 1) throw std::runtime_error("N must be >= 1");
        p_.N = N;
        u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
      } else if (match("dt")) {
        double dt = p.as_double();
        if (dt <= 0.0) throw std::runtime_error("dt must be > 0");
        p_.dt = dt;
      } else if (match("v_max")) {
        p_.v_max = p.as_double();
        if (!has_speed_limit_) v_max_eff_ = p_.v_max;
      } else if (match("v_min")) p_.v_min = p.as_double();
      else if (match("w_max")) p_.w_max = p.as_double();
      else if (match("dv_max")) p_.dv_max = p.as_double();
      else if (match("dw_max")) p_.dw_max = p.as_double();

      else if (match("w_pos")) p_.w_pos = p.as_double();
      else if (match("w_yaw")) p_.w_yaw = p.as_double();
      else if (match("w_v")) p_.w_v = p.as_double();
      else if (match("w_w")) p_.w_w = p.as_double();
      else if (match("w_dv")) p_.w_dv = p.as_double();
      else if (match("w_dw")) p_.w_dw = p.as_double();

      else if (match("w_terminal_pos")) p_.w_terminal_pos = p.as_double();
      else if (match("w_terminal_yaw")) p_.w_terminal_yaw = p.as_double();

      else if (match("w_progress")) p_.w_progress = p.as_double();
      else if (match("v_ref")) p_.v_ref = p.as_double();
      else if (match("progress_stop_dist")) p_.progress_stop_dist = p.as_double();
      else if (match("yaw_stop_dist")) p_.yaw_stop_dist = p.as_double();

      else if (match("w_costmap")) p_.w_costmap = p.as_double();
      else if (match("lethal_cost")) p_.lethal_cost = p.as_double();

      else if (match("opt_iters")) p_.opt_iters = p.as_int();
      else if (match("step_size")) p_.step_size = p.as_double();
      else if (match("grad_eps")) p_.grad_eps = p.as_double();

      else if (match("prune_dist")) p_.prune_dist = p.as_double();
      else if (match("ref_stride")) p_.ref_stride = p.as_int();
      else if (match("transform_tolerance")) p_.transform_tolerance = p.as_double();

      else if (match("cmd_smoothing")) p_.cmd_smoothing = p.as_double();
      else if (match("publish_debug")) p_.publish_debug = p.as_bool();
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
  }

  return result;
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();

  cacheCycleTransform_(cmd.header.stamp);
  if (!have_cycle_tf_) {
    throw nav2_core::ControllerException("MPC: missing global<-base TF for costmap evaluation");
  }

  if (!has_plan_ || global_plan_.poses.empty()) {
    throw nav2_core::ControllerException("MPC: No plan set");
  }

  // Stop if goal reached
  if (goal_checker) {
    if (goal_checker->isGoalReached(pose.pose, goal_pose_.pose, velocity)) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;
      return cmd;
    }
  }

  // Transform plan to base frame
  nav_msgs::msg::Path local_plan = transformPlanToBaseFrame(global_plan_, pose);
  if (local_plan.poses.size() < 2) {
    throw nav2_core::ControllerException("MPC: Local plan too small after transform");
  }

  // Publish transformed local plan
  if (p_.publish_debug && pub_local_plan_ && pub_local_plan_->is_activated()) {
    local_plan.header.stamp = clock_->now();
    pub_local_plan_->publish(local_plan);
  }

  // Build reference horizon in base frame
  Eigen::MatrixXd xref;
  buildReferenceHorizon(local_plan, p_.N, xref);

  // Base-frame state: robot at origin in its own frame
  const Eigen::Vector3d x0(0.0, 0.0, 0.0);

  // Receding horizon warm-start shift
  if (u_seq_.rows() == p_.N) {
    for (int k = 0; k < p_.N - 1; ++k) {
      u_seq_.row(k) = u_seq_.row(k + 1);
    }
    u_seq_.row(p_.N - 1) = u_seq_.row(p_.N - 2);
  } else {
    u_seq_ = Eigen::MatrixXd::Zero(p_.N, 2);
  }

  const Eigen::Vector2d u_prev(last_u_(0), last_u_(1));

  // Optimize
  optimizeControlSequence(x0, xref, u_prev, u_seq_);

  // Take first control
  Eigen::Vector2d u0(u_seq_(0, 0), u_seq_(0, 1));
  u0 = clampControl(u0);
  u0 = clampAccel(u0, u_prev);

  // Basic safety: check first-step predicted collision
  const Eigen::Vector3d x1 = stepDiffDrive(x0, u0, p_.dt);
  if (inCollisionAtBasePose(x1)) {
    throw nav2_core::NoValidControl("MPC: First-step command results in collision");
  }

  // Command smoothing (a in [0..1], higher => smoother but slower response)
  const double a = std::max(0.0, std::min(1.0, p_.cmd_smoothing));
  const Eigen::Vector2d u_smoothed = a * last_u_ + (1.0 - a) * u0;

  last_u_ = u_smoothed;

  cmd.twist.linear.x = u_smoothed(0);
  cmd.twist.angular.z = u_smoothed(1);

  // Publish predicted path
  if (p_.publish_debug && pub_pred_path_ && pub_pred_path_->is_activated()) {
    pub_pred_path_->publish(makePredictedPathMsg(x0, u_seq_, clock_->now()));
  }

  return cmd;
}

}  // namespace nav2_mpc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MpcController, nav2_core::Controller)
