#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

#include <Eigen/Dense>

namespace nav2_mpc_controller
{

class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Params
  {
    // MPC timing
    int N{15};
    double dt{0.1};

    // Limits
    double v_max{0.5};
    double v_min{-0.1};
    double w_max{1.5};

    // Optional accel (soft cost OR hard clamp)
    double dv_max{0.5};
    double dw_max{2.5};

    double progress_stop_dist{0.6};  // meters: disable progress reward near goal/target

    // Cost weights
    double w_pos{5.0};
    double w_yaw{2.0};
    double w_v{0.2};
    double w_w{0.2};
    double w_dv{0.5};
    double w_dw{0.5};

    double w_terminal_pos{30.0};
    double w_terminal_yaw{2.0};

    double yaw_stop_dist{0.5};

    // Progress / forward motion encouragement
    double w_progress{0.5};   // weight for forward progress
    double v_ref{0.3};        // desired forward speed (m/s)

    // Collision / costmap weighting
    double w_costmap{10.0};          // penalty weight for non-lethal costs
    double lethal_cost{253.0};       // treat >= as collision (usually 253/254/255)

    // Optimization
    int opt_iters{20};
    double step_size{0.3};
    double grad_eps{1e-3};

    // Path handling
    double prune_dist{2.5};
    int ref_stride{1};
    double transform_tolerance{0.2};

    // Output smoothing
    double cmd_smoothing{0.2};

    // Debug
    bool publish_debug{true};
  };

  // --- internal helpers ---
  void cacheCycleTransform_(const rclcpp::Time & stamp);

  nav_msgs::msg::Path transformPlanToBaseFrame(
    const nav_msgs::msg::Path & plan,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  void buildReferenceHorizon(
    const nav_msgs::msg::Path & local_plan_base,
    int N,
    Eigen::MatrixXd & xref /* Nx3 */) const;

  static Eigen::Vector3d stepDiffDrive(
    const Eigen::Vector3d & x,
    const Eigen::Vector2d & u,
    double dt);

  Eigen::Vector2d clampControl(const Eigen::Vector2d & u) const;

  // Optional: clamp accel relative to previous command (hard limit)
  Eigen::Vector2d clampAccel(
    const Eigen::Vector2d & u,
    const Eigen::Vector2d & u_prev) const;

  // Collision/costmap evaluation of a pose in base frame
  double costmapCostAtBasePose(const Eigen::Vector3d & x_base) const;
  bool inCollisionAtBasePose(const Eigen::Vector3d & x_base) const;

  double rolloutCost(
    const Eigen::Vector3d & x0,
    const Eigen::MatrixXd & u_seq /* Nx2 */,
    const Eigen::MatrixXd & xref  /* Nx3 */,
    const Eigen::Vector2d & u_prev) const;

  void optimizeControlSequence(
    const Eigen::Vector3d & x0,
    const Eigen::MatrixXd & xref,
    const Eigen::Vector2d & u_prev,
    Eigen::MatrixXd & u_seq_io);

  nav_msgs::msg::Path makePredictedPathMsg(
    const Eigen::Vector3d & x0,
    const Eigen::MatrixXd & u_seq,
    const rclcpp::Time & stamp) const;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_mpc_controller")};
  rclcpp::Clock::SharedPtr clock_;

  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  Params p_;

  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_plan_{false};

  Eigen::MatrixXd u_seq_;
  Eigen::Vector2d last_u_{0.0, 0.0};

  bool has_speed_limit_{false};
  double v_max_eff_{0.5};

  // Cached base->global transform for the current control cycle (fast costmap queries)
  bool have_cycle_tf_{false};
  double cycle_gx_{0.0};    // base origin in global frame
  double cycle_gy_{0.0};
  double cycle_gyaw_{0.0};  // base yaw in global frame
  double cycle_c_{1.0};     // cos(cycle_gyaw_)
  double cycle_s_{0.0};     // sin(cycle_gyaw_)


  // Collision checker
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>> collision_checker_;

  // Dynamic params
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Debug publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_local_plan_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_pred_path_;
};

}  // namespace nav2_mpc_controller
