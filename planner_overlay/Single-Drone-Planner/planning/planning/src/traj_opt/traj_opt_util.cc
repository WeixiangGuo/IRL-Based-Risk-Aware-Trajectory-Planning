#include <traj_opt/lbfgs_raw.hpp>
#include <traj_opt/traj_opt.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace traj_opt {

using rot_util = rotation_util::RotUtil;

TrajOpt::TrajOpt(ros::NodeHandle &nh,
                 std::shared_ptr<parameter_server::ParaeterSerer> &paraPtr) {

  nh_ = nh;
  paraPtr->get_para("K", K_);
  paraPtr->get_para("max_iter", max_iter_);

  paraPtr->get_para("vmax", vmax_);
  v_max_tail_ = vmax_;

  // paraPtr->get_para("dv_max_horiz_track", dv_max_horiz_track_);
  // paraPtr->get_para("dv_max_horiz_land", dv_max_horiz_land_);
  // paraPtr->get_para("v_max_vert_track", v_max_vert_track_);
  // paraPtr->get_para("v_max_vert_land", v_max_vert_land_);

  paraPtr->get_para("amax", amax_);
  a_max_tail_ = amax_;
  paraPtr->get_para("dyaw_max", dyaw_max_);
  paraPtr->get_para("thrust_max", thrust_max_);
  paraPtr->get_para("thrust_min", thrust_min_);
  paraPtr->get_para("omega_max", omega_max_);
  paraPtr->get_para("omega_yaw_max", omega_yaw_max_);
  // paraPtr->get_para("v_plus", v_plus_);
  // paraPtr->get_para("robot_l", robot_l_);
  // paraPtr->get_para("robot_r", robot_r_);
  // paraPtr->get_para("platform_r", platform_r_);
  // paraPtr->get_para("preception_d_max", preception_d_max_);
  // paraPtr->get_para("preception_d_min", preception_d_min_);
  // paraPtr->get_para("land_z_down_relative", land_z_down_relative_);
  // paraPtr->get_para("land_z_up_relative", land_z_up_relative_);
  // paraPtr->get_para("short_mode_with_perception", short_mode_with_perception_);

  paraPtr->get_para("rhoT", rhoT_);
  // paraPtr->get_para("rhoT_track", rhoT_track_);
  // paraPtr->get_para("rhoT_land", rhoT_land_origin_);
  // rhoT_land_ = rhoT_land_origin_;
  // paraPtr->get_para("rhoVt", rhoVt_);
  // paraPtr->get_para("rhoVt", rhoAt_);
  // paraPtr->get_para("rhoPt", rhoPt_);

  paraPtr->get_para("rhoWP", rhoWP_);
  paraPtr->get_para("rhoRotFactor", rhoRotFactor_);
  paraPtr->get_para("rhoP", rhoP_);
  paraPtr->get_para("rhoV", rhoV_);
  paraPtr->get_para("rhoA", rhoA_);
  rhoV_tail_ = rhoV_;
  rhoA_tail_ = rhoA_;
  paraPtr->get_para("rhoThrust", rhoThrust_);
  paraPtr->get_para("rhoOmega", rhoOmega_);
  paraPtr->get_para("rhoPerchingCollision", rhoPerchingCollision_);
  paraPtr->get_para("rhoPerchingPreception", rhoPerchingPreception_);

  paraPtr->get_para("rhoYaw", rhoYaw_);
  std::cout << "rhoYaw: " << rhoYaw_ << std::endl;
  paraPtr->get_para("rhoDyaw", rhoDyaw_);

  // Theta (arm joint) parameters
  double theta_0_min, theta_0_max, theta_1_min, theta_1_max, theta_2_min,
      theta_2_max;
  double dtheta_0_max, dtheta_1_max, dtheta_2_max;
  paraPtr->get_para("theta_0_min", theta_0_min);
  paraPtr->get_para("theta_0_max", theta_0_max);
  paraPtr->get_para("theta_1_min", theta_1_min);
  paraPtr->get_para("theta_1_max", theta_1_max);
  paraPtr->get_para("theta_2_min", theta_2_min);
  paraPtr->get_para("theta_2_max", theta_2_max);
  theta_min_ << theta_0_min, theta_1_min, theta_2_min;
  theta_max_ << theta_0_max, theta_1_max, theta_2_max;

  paraPtr->get_para("dtheta_0_max", dtheta_0_max);
  paraPtr->get_para("dtheta_1_max", dtheta_1_max);
  paraPtr->get_para("dtheta_2_max", dtheta_2_max);
  dtheta_max_ << dtheta_0_max, dtheta_1_max, dtheta_2_max;

  paraPtr->get_para("rhoTheta", rhoTheta_);
  paraPtr->get_para("rhoDtheta", rhoDtheta_);

  int enable_astar_waypoint_tracking = 0;
  paraPtr->get_para("astar_waypoint_tracking_enable",
                    enable_astar_waypoint_tracking);
  enable_astar_waypoint_tracking_ = enable_astar_waypoint_tracking != 0;
  paraPtr->get_para("rhoAstarWaypoint", rhoAstarWaypoint_);
  const double rho_astar_waypoint_config = rhoAstarWaypoint_;
  ros::param::param("~rhoAstarWaypoint", rhoAstarWaypoint_,
                    rhoAstarWaypoint_);
  ros::param::param("~rhoAstarWaypointStage2", rhoAstarWaypointStage2_, -1.0);
  if (rhoAstarWaypointStage2_ < 0.0) {
    rhoAstarWaypointStage2_ = rhoAstarWaypoint_;
  }
  if (std::abs(rhoAstarWaypoint_ - rho_astar_waypoint_config) > 1e-9) {
    ROS_INFO("[TrajOpt] rhoAstarWaypoint override: config=%.3f ros=%.3f",
             rho_astar_waypoint_config, rhoAstarWaypoint_);
  }
  ROS_INFO("[TrajOpt] A* waypoint tracking enable=%d rho_stage1=%.3f rho_stage2=%.3f",
           enable_astar_waypoint_tracking_ ? 1 : 0,
           rhoAstarWaypoint_, rhoAstarWaypointStage2_);

  ros::param::param("~mincoWarmStartCacheEnable",
                    minco_warm_start_cache_enable_,
                    minco_warm_start_cache_enable_);
  ros::param::param("~mincoWarmStartCacheForceReuse",
                    minco_warm_start_cache_force_reuse_,
                    minco_warm_start_cache_force_reuse_);
  ros::param::param("~mincoWarmStartCacheTolerance",
                    minco_warm_start_cache_tol_,
                    minco_warm_start_cache_tol_);
  ROS_INFO("[TrajOpt] warm-start cache enable=%d force_reuse=%d tol=%.3e",
           minco_warm_start_cache_enable_ ? 1 : 0,
           minco_warm_start_cache_force_reuse_ ? 1 : 0,
           minco_warm_start_cache_tol_);

  paraPtr->get_para("pause_debug", pause_debug_);
  paraPtr->get_para("short_mode_time", short_mode_time_);

  paraPtr->get_para("eps_pz", eps_pz_);

  paraPtr->get_para("dist_threshold", dist_threshold_);
  paraPtr->get_para("dist_threshold_end", dist_threshold_end_);
  paraPtr->get_para("dist_threshold_end_range", dist_threshold_end_range_);
  int enable_irl_sphere_collision = 0;
  paraPtr->get_para("irl_sphere_collision_enable", enable_irl_sphere_collision);
  enable_irl_sphere_collision_ = enable_irl_sphere_collision != 0;
  paraPtr->get_para("irl_uam_sphere_radius", irl_uam_sphere_radius_);
  paraPtr->get_para("irl_safety_distance_1", irl_safety_distance_1_);
  paraPtr->get_para("irl_safety_distance_2", irl_safety_distance_2_);
  ros::param::param("~irl_collision_weight_scale",
                    irl_collision_weight_scale_,
                    irl_collision_weight_scale_);
  irl_collision_weight_scale_ = std::max(0.0, irl_collision_weight_scale_);
  ROS_INFO("[TrajOpt][IRL] collision weight scale=%.3f",
           irl_collision_weight_scale_);
  ros::param::param("~irl_obs_optimization_margin",
                    irl_obs_optimization_margin_,
                    irl_obs_optimization_margin_);
  irl_obs_optimization_margin_ =
      std::max(0.0, irl_obs_optimization_margin_);
  ROS_INFO("[TrajOpt][IRL] obs optimization margin=%.3f hard check margin=0.000",
           irl_obs_optimization_margin_);
  int enable_irl_analytic_cylinder_collision = 0;
  paraPtr->get_para("irl_analytic_cylinder_enable",
                    enable_irl_analytic_cylinder_collision);
  bool enable_analytic_cylinder =
      enable_irl_analytic_cylinder_collision != 0;
  ros::param::param("~irl_analytic_cylinder_enable",
                    enable_analytic_cylinder, enable_analytic_cylinder);
  enable_irl_analytic_cylinder_collision_ = enable_analytic_cylinder;
  paraPtr->get_para("irl_cylinder_1_x", irl_cylinder1_center_xy_.x());
  paraPtr->get_para("irl_cylinder_1_y", irl_cylinder1_center_xy_.y());
  paraPtr->get_para("irl_cylinder_2_x", irl_cylinder2_center_xy_.x());
  paraPtr->get_para("irl_cylinder_2_y", irl_cylinder2_center_xy_.y());
  paraPtr->get_para("irl_cylinder_radius", irl_cylinder_radius_);
  irl_cylinder1_radius_ = irl_cylinder_radius_;
  irl_cylinder2_radius_ = irl_cylinder_radius_;
  paraPtr->get_para("irl_cylinder_1_radius", irl_cylinder1_radius_);
  paraPtr->get_para("irl_cylinder_2_radius", irl_cylinder2_radius_);
  paraPtr->get_para("irl_cylinder_z_min", irl_cylinder_z_min_);
  paraPtr->get_para("irl_cylinder_z_max", irl_cylinder_z_max_);
  ros::param::param("~irl_cylinder_1_x", irl_cylinder1_center_xy_.x(),
                    irl_cylinder1_center_xy_.x());
  ros::param::param("~irl_cylinder_1_y", irl_cylinder1_center_xy_.y(),
                    irl_cylinder1_center_xy_.y());
  ros::param::param("~irl_cylinder_2_x", irl_cylinder2_center_xy_.x(),
                    irl_cylinder2_center_xy_.x());
  ros::param::param("~irl_cylinder_2_y", irl_cylinder2_center_xy_.y(),
                    irl_cylinder2_center_xy_.y());
  ros::param::param("~irl_cylinder_radius", irl_cylinder_radius_,
                    irl_cylinder_radius_);
  ros::param::param("~irl_cylinder_1_radius", irl_cylinder1_radius_,
                    irl_cylinder_radius_);
  ros::param::param("~irl_cylinder_2_radius", irl_cylinder2_radius_,
                    irl_cylinder_radius_);
  irl_cylinder1_radius_ = std::max(irl_cylinder1_radius_, 1e-6);
  irl_cylinder2_radius_ = std::max(irl_cylinder2_radius_, 1e-6);
  ros::param::param("~irl_cylinder_z_min", irl_cylinder_z_min_,
                    irl_cylinder_z_min_);
  ros::param::param("~irl_cylinder_z_max", irl_cylinder_z_max_,
                    irl_cylinder_z_max_);
  if (irl_cylinder_z_max_ < irl_cylinder_z_min_) {
    std::swap(irl_cylinder_z_min_, irl_cylinder_z_max_);
  }
  ROS_INFO("[TrajOpt][IRL] analytic cylinder enable=%d c1=(%.3f, %.3f) "
           "r1=%.3f c2=(%.3f, %.3f) r2=%.3f z=[%.3f, %.3f]",
           enable_irl_analytic_cylinder_collision_ ? 1 : 0,
           irl_cylinder1_center_xy_.x(), irl_cylinder1_center_xy_.y(),
           irl_cylinder1_radius_,
           irl_cylinder2_center_xy_.x(), irl_cylinder2_center_xy_.y(),
           irl_cylinder2_radius_, irl_cylinder_z_min_, irl_cylinder_z_max_);

  ros::param::param("~irl_minco_debug_enable", irl_minco_debug_enable_, false);
  ros::param::param("~irl_minco_debug_print_stride",
                    irl_minco_debug_print_stride_,
                    irl_minco_debug_print_stride_);
  ros::param::param("~irl_minco_debug_marker_stride",
                    irl_minco_debug_marker_stride_,
                    irl_minco_debug_marker_stride_);
  ros::param::param("~irl_minco_debug_max_markers",
                    irl_minco_debug_max_markers_,
                    irl_minco_debug_max_markers_);
  ros::param::param("~irl_minco_debug_sample_dt",
                    irl_minco_debug_sample_dt_,
                    irl_minco_debug_sample_dt_);
  irl_minco_debug_print_stride_ = std::max(1, irl_minco_debug_print_stride_);
  irl_minco_debug_marker_stride_ = std::max(1, irl_minco_debug_marker_stride_);
  irl_minco_debug_max_markers_ = std::max(1, irl_minco_debug_max_markers_);
  irl_minco_debug_sample_dt_ = std::max(0.01, irl_minco_debug_sample_dt_);
  ROS_INFO("[TrajOpt][IRL] minco debug enable=%d print_stride=%d "
           "marker_stride=%d max_markers=%d sample_dt=%.3f",
           irl_minco_debug_enable_ ? 1 : 0, irl_minco_debug_print_stride_,
           irl_minco_debug_marker_stride_, irl_minco_debug_max_markers_,
           irl_minco_debug_sample_dt_);
  end_pos_goal_.setZero();
  paraPtr->get_para("enable_tail_constraint", en_tail_constraint_);
  {
    const bool enable_tail_constraint_config = en_tail_constraint_;
    bool enable_tail_constraint_ros = en_tail_constraint_;
    ros::param::param("~enable_tail_constraint", enable_tail_constraint_ros,
                      enable_tail_constraint_ros);
    if (enable_tail_constraint_ros != en_tail_constraint_) {
      ROS_INFO("[TrajOpt] enable_tail_constraint override: config=%d ros=%d",
               enable_tail_constraint_config ? 1 : 0,
               enable_tail_constraint_ros ? 1 : 0);
    }
    en_tail_constraint_ = enable_tail_constraint_ros;
  }
  paraPtr->get_para("tail_offset", tail_offset_);
  paraPtr->get_para("v_max_tail", v_max_tail_);
  paraPtr->get_para("a_max_tail", a_max_tail_);
  paraPtr->get_para("rhoV_tail", rhoV_tail_);
  paraPtr->get_para("rhoA_tail", rhoA_tail_);

  cam2body_R_down_ << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;

  cam2body_p_down_ << 0.0, 0.0, 0.0;
  fx_down_ = 320;
  fy_down_ = 320;
  cx_down_ = 320;
  cy_down_ = 240;

  cam2body_R_front_ << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;

  cam2body_p_front_ << 0.0, 0.0, 0.0;
  fx_front_ = 240;
  fy_front_ = 240;
  cx_front_ = 240;
  cy_front_ = 240;

  pub_debug_opt_ = nh_.advertise<quadrotor_msgs::DebugOpt>("debug_opt", 1);
  pub_irl_minco_debug_markers_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "irl_minco_inner_iterations", 1, true);
}

void TrajOpt::set_irl_sphere_safety_distances(const double &d1,
                                              const double &d2,
                                              const double &sphere_radius) {
  std::lock_guard<std::mutex> lock(irl_sphere_collision_mutex_);
  irl_safety_distance_1_ = std::max(0.0, d1);
  irl_safety_distance_2_ = std::max(0.0, d2);
  if (sphere_radius > 0.0) {
    irl_uam_sphere_radius_ = sphere_radius;
  }
  ROS_INFO("[TrajOpt][IRL] sphere collision d1=%.3f d2=%.3f radius=%.3f",
           irl_safety_distance_1_, irl_safety_distance_2_,
           irl_uam_sphere_radius_);
}

static bool cylinderSignedDistanceAndGrad(const Eigen::Vector3d &p,
                                          const Eigen::Vector2d &center_xy,
                                          const double cylinder_radius,
                                          const double z_min,
                                          const double z_max,
                                          double &signed_distance,
                                          Eigen::Vector3d *grad) {
  const double radius = std::max(cylinder_radius, 1e-6);
  const double z_low = std::min(z_min, z_max);
  const double z_high = std::max(z_min, z_max);
  const Eigen::Vector2d dxy = p.head<2>() - center_xy;
  const double radial = dxy.norm();
  Eigen::Vector2d radial_dir(1.0, 0.0);
  if (radial > 1e-8) {
    radial_dir = dxy / radial;
  }

  const double side = radial - radius;
  const double below = z_low - p.z();
  const double above = p.z() - z_high;

  if (side > 0.0 || below > 0.0 || above > 0.0) {
    const double radial_excess = std::max(side, 0.0);
    double z_excess = 0.0;
    if (below > 0.0) {
      z_excess = -below;
    } else if (above > 0.0) {
      z_excess = above;
    }

    signed_distance =
        std::sqrt(radial_excess * radial_excess + z_excess * z_excess);
    if (grad) {
      if (signed_distance > 1e-8) {
        *grad = Eigen::Vector3d(radial_excess * radial_dir.x(),
                                radial_excess * radial_dir.y(), z_excess) /
                signed_distance;
      } else {
        *grad = Eigen::Vector3d(radial_dir.x(), radial_dir.y(), 0.0);
      }
    }
    return true;
  }

  const double inside_side = side;
  const double inside_bottom = p.z() - z_low;
  const double inside_top = z_high - p.z();
  const double signed_bottom = -inside_bottom;
  const double signed_top = -inside_top;
  signed_distance = std::max(inside_side, std::max(signed_bottom, signed_top));

  if (grad) {
    if (signed_distance == inside_side) {
      *grad = Eigen::Vector3d(radial_dir.x(), radial_dir.y(), 0.0);
    } else if (signed_distance == signed_bottom) {
      *grad = Eigen::Vector3d(0.0, 0.0, -1.0);
    } else {
      *grad = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
  }
  return true;
}

void TrajOpt::set_astar_tracking_path(
    const std::vector<Eigen::Vector3d> &path) {
  astar_tracking_wps_.clear();
  astar_tracking_wps_.reserve(path.size());

  for (const Eigen::Vector3d &p : path) {
    if (astar_tracking_wps_.empty() ||
        (p - astar_tracking_wps_.back()).norm() > 1e-3) {
      astar_tracking_wps_.push_back(p);
    }
  }

  ROS_INFO("[TrajOpt] A* waypoint tracking path points=%zu",
           astar_tracking_wps_.size());
}

bool TrajOpt::grad_cost_astar_waypoint_tracking(const Eigen::Vector3d &p,
                                                Eigen::Vector3d &gradp,
                                                double &costp) {
  gradp.setZero();
  costp = 0.0;

  const double rho = is_warm_opt_ ? rhoAstarWaypoint_ : rhoAstarWaypointStage2_;
  if (!enable_astar_waypoint_tracking_ || rho <= 0.0 ||
      astar_tracking_wps_.size() < 2) {
    return false;
  }

  double best_sq_dist = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_ref = astar_tracking_wps_.front();

  for (size_t i = 0; i + 1 < astar_tracking_wps_.size(); ++i) {
    const Eigen::Vector3d &a = astar_tracking_wps_[i];
    const Eigen::Vector3d &b = astar_tracking_wps_[i + 1];
    const Eigen::Vector3d ab = b - a;
    const double ab_sq_norm = ab.squaredNorm();

    Eigen::Vector3d ref = a;
    if (ab_sq_norm > 1e-8) {
      double s = (p - a).dot(ab) / ab_sq_norm;
      if (s < 0.0) {
        s = 0.0;
      } else if (s > 1.0) {
        s = 1.0;
      }
      ref = a + s * ab;
    }

    const double sq_dist = (p - ref).squaredNorm();
    if (sq_dist < best_sq_dist) {
      best_sq_dist = sq_dist;
      best_ref = ref;
    }
  }

  if (!std::isfinite(best_sq_dist) || best_sq_dist < 1e-12) {
    return false;
  }

  const Eigen::Vector3d dp = p - best_ref;
  costp = 0.5 * rho * best_sq_dist;
  gradp = rho * dp;
  return true;
}

bool TrajOpt::is_irl_sphere_traj_valid(const Trajectory<7> &traj,
                                       const double sample_dt,
                                       double *min_margin,
                                       Eigen::Vector3d *min_margin_pos,
                                       int *min_margin_group) {
  bool enable_irl_sphere_collision = false;
  bool enable_irl_analytic_cylinder_collision = false;
  double sphere_radius = 0.0;
  double safety_distance_1 = 0.0;
  double safety_distance_2 = 0.0;
  Eigen::Vector2d cylinder1_center_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d cylinder2_center_xy = Eigen::Vector2d::Zero();
  double cylinder1_radius = 0.0;
  double cylinder2_radius = 0.0;
  double cylinder_z_min = 0.0;
  double cylinder_z_max = 0.0;
  {
    std::lock_guard<std::mutex> lock(irl_sphere_collision_mutex_);
    enable_irl_sphere_collision = enable_irl_sphere_collision_;
    enable_irl_analytic_cylinder_collision =
        enable_irl_analytic_cylinder_collision_;
    sphere_radius = irl_uam_sphere_radius_;
    safety_distance_1 = irl_safety_distance_1_;
    safety_distance_2 = irl_safety_distance_2_;
    cylinder1_center_xy = irl_cylinder1_center_xy_;
    cylinder2_center_xy = irl_cylinder2_center_xy_;
    cylinder1_radius = irl_cylinder1_radius_;
    cylinder2_radius = irl_cylinder2_radius_;
    cylinder_z_min = irl_cylinder_z_min_;
    cylinder_z_max = irl_cylinder_z_max_;
  }

  if (!enable_irl_sphere_collision) {
    if (min_margin) {
      *min_margin = std::numeric_limits<double>::infinity();
    }
    if (min_margin_pos) {
      min_margin_pos->setZero();
    }
    if (min_margin_group) {
      *min_margin_group = 0;
    }
    return true;
  }

  if (!enable_irl_analytic_cylinder_collision &&
      aabb_pts_group1_vec_.empty() && aabb_pts_group2_vec_.empty()) {
    ROS_WARN_THROTTLE(1.0,
                      "[TrajOpt][IRL] sphere hard check failed: no obstacle "
                      "points cached.");
    if (min_margin) {
      *min_margin = -std::numeric_limits<double>::infinity();
    }
    if (min_margin_pos) {
      min_margin_pos->setZero();
    }
    if (min_margin_group) {
      *min_margin_group = 0;
    }
    return false;
  }

  const double dt = sample_dt > 1e-4 ? sample_dt : 0.01;
  const double required_clearance_1 = sphere_radius + safety_distance_1;
  const double required_clearance_2 = sphere_radius + safety_distance_2;
  double worst_margin = std::numeric_limits<double>::infinity();
  double worst_t = 0.0;
  Eigen::Vector3d worst_pos = Eigen::Vector3d::Zero();
  int worst_group = 0;

  auto update_margin =
      [&](const Eigen::Vector3d &pos, const double traj_t,
          const std::vector<Eigen::Vector3d> &obs_pts,
          const double required_clearance, const int group_id) {
        if (obs_pts.empty()) {
          return;
        }

        double min_sq_dist = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector3d &obs : obs_pts) {
          const double sq_dist = (pos - obs).squaredNorm();
          if (sq_dist < min_sq_dist) {
            min_sq_dist = sq_dist;
          }
        }

        if (!std::isfinite(min_sq_dist)) {
          return;
        }

        const double margin = std::sqrt(min_sq_dist) - required_clearance;
        if (margin < worst_margin) {
          worst_margin = margin;
          worst_t = traj_t;
          worst_pos = pos;
          worst_group = group_id;
        }
      };

  auto update_cylinder_margin =
      [&](const Eigen::Vector3d &pos, const double traj_t,
          const Eigen::Vector2d &center_xy, const double cylinder_radius,
          const double required_clearance, const int group_id) {
        double signed_distance = 0.0;
        if (!cylinderSignedDistanceAndGrad(pos, center_xy, cylinder_radius,
                                           cylinder_z_min, cylinder_z_max,
                                           signed_distance, nullptr)) {
          return;
        }

        const double margin = signed_distance - required_clearance;
        if (margin < worst_margin) {
          worst_margin = margin;
          worst_t = traj_t;
          worst_pos = pos;
          worst_group = group_id;
        }
      };

  double traj_t_offset = 0.0;
  int piece_id = 0;
  for (const auto &piece : traj) {
    const double dur = piece.getDuration();
    int steps = static_cast<int>(std::ceil(dur / dt));
    if (steps < 1) {
      steps = 1;
    }

    for (int i = 0; i <= steps; ++i) {
      if (piece_id > 0 && i == 0) {
        continue;
      }
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      const double t = dur * ratio;
      const Eigen::Vector3d pos = piece.getPos(t);
      const double traj_t = traj_t_offset + t;

      if (enable_irl_analytic_cylinder_collision) {
        update_cylinder_margin(pos, traj_t, cylinder1_center_xy,
                               cylinder1_radius, required_clearance_1, 1);
        update_cylinder_margin(pos, traj_t, cylinder2_center_xy,
                               cylinder2_radius, required_clearance_2, 2);
      } else {
        update_margin(pos, traj_t, aabb_pts_group1_vec_, required_clearance_1,
                      1);
        update_margin(pos, traj_t, aabb_pts_group2_vec_, required_clearance_2,
                      2);
      }
    }

    traj_t_offset += dur;
    ++piece_id;
  }

  if (min_margin) {
    *min_margin = worst_margin;
  }
  if (min_margin_pos) {
    *min_margin_pos = worst_pos;
  }
  if (min_margin_group) {
    *min_margin_group = worst_group;
  }

  if (!std::isfinite(worst_margin)) {
    ROS_WARN_THROTTLE(1.0,
                      "[TrajOpt][IRL] sphere hard check failed: no finite "
                      "distance found.");
    return false;
  }

  if (worst_margin < 0.0) {
    INFO_MSG_RED("[TrajOpt][IRL] sphere hard check failed, group="
                 << worst_group << ", t=" << worst_t
                 << ", margin=" << worst_margin
                 << ", pos=" << worst_pos.transpose());
    return false;
  }

  ROS_INFO_THROTTLE(1.0,
                    "[TrajOpt][IRL] sphere hard check pass, min_margin=%.3f, "
                    "group=%d",
                    worst_margin, worst_group);
  return true;
}

void TrajOpt::append_tail_constraint_mid_points(
    const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
    const Eigen::MatrixXd &final_yaw, const Eigen::MatrixXd &final_thetas,
    std::vector<Eigen::Vector3d> &mid_q_vec, int &N_ret) {
  Eigen::Matrix4d T_e2b;
  Eigen::Quaterniond q_body_tail(
      Eigen::AngleAxisd(final_yaw(0, 0), Eigen::Vector3d::UnitZ()));
  rc_sdf_ptr_->kine_ptr_->getEndPose(finState.col(0), q_body_tail,
                                     final_thetas.col(0), p_end_end_,
                                     q_end_end_, T_e2b);
  Eigen::Matrix3d R_end_end = q_end_end_.toRotationMatrix();
  double offset = tail_offset_;
  p_end_tail_ = p_end_end_ - offset * R_end_end.col(0);
  p_body_tail_ = finState.col(0) - offset * R_end_end.col(0);
  thetas_tail_ = final_thetas.col(0);
  q_body_tail_ = q_body_tail;

  visualization_msgs::MarkerArray robot_markers;
  rc_sdf_ptr_->getRobotMarkerArray(p_body_tail_, q_body_tail, final_thetas.col(0),
                                   robot_markers, visualization_rc_sdf::Color::red);
  rc_sdf_ptr_->robotMarkersPub(robot_markers, "tail_robot_marker");

  //* method 1 straight line mid points
  // mid_q_vec.push_back(iniState.col(0) + 0.33 * (p_body_tail_ - iniState.col(0)));
  // mid_q_vec.push_back(iniState.col(0) + 0.66 * (p_body_tail_ - iniState.col(0)));

  //* method 2 astar path
  mid_q_vec.push_back(p_body_tail_);
  mid_q_vec.push_back(0.5 * (p_body_tail_ + finState.col(0)));
  N_ret = mid_q_vec.size() + 1;
}

void TrajOpt::append_ring_mid_points(const Eigen::MatrixXd &iniState,
                                     std::vector<Eigen::Vector3d> &mid_q_vec,
                                     double offset) {
  Eigen::Vector3d ring_center;
  ring_center << ring_pose_.pose.position.x, ring_pose_.pose.position.y,
      ring_pose_.pose.position.z;
  Eigen::Quaterniond ring_q(ring_pose_.pose.orientation.w,
                            ring_pose_.pose.orientation.x,
                            ring_pose_.pose.orientation.y,
                            ring_pose_.pose.orientation.z);
  Eigen::Vector3d x_dir = ring_q * Eigen::Vector3d::UnitX();
  Eigen::Vector3d z_dir = ring_q * Eigen::Vector3d::UnitZ();
  Eigen::Vector3d init_pos = iniState.col(0);
  double dir = (init_pos - ring_center).dot(x_dir) >= 0.0 ? 1.0 : -1.0;
  ring_mid_pts_.clear();
  ring_mid_pts_.push_back(ring_center + dir * offset * x_dir);
  ring_mid_pts_.push_back(ring_center);
  ring_mid_pts_.push_back(ring_center - dir * offset * x_dir);
  // align ring orientation so traversal direction is +X
  Eigen::Vector3d x_axis = (-dir) * x_dir;
  if (x_axis.norm() < 1e-6) {
    x_axis = Eigen::Vector3d::UnitX();
  }
  x_axis.normalize();
  Eigen::Vector3d z_axis = z_dir - z_dir.dot(x_axis) * x_axis;
  if (z_axis.norm() < 1e-6) {
    z_axis = Eigen::Vector3d::UnitZ();
  }
  z_axis.normalize();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  if (y_axis.norm() < 1e-6) {
    y_axis = Eigen::Vector3d::UnitY();
  }
  y_axis.normalize();
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
  ring_mid_quat_ = Eigen::Quaterniond(R);
  has_ring_mid_quat_ = true;
  mid_q_vec.insert(mid_q_vec.end(), ring_mid_pts_.begin(), ring_mid_pts_.end());

  
}

double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}

double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

double smoothedL1(const double &x, double &grad) {
  static double mu = 0.01;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return mumxd2 * sqrxdmu * xdmu;
  }
}

bool smoothedL1(const double &x, const double &mu, double &f, double &df) {
  if (x < 0.0) {
    return false;
  } else if (x > mu) {
    f = x - 0.5 * mu;
    df = 1.0;
    return true;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    f = mumxd2 * sqrxdmu * xdmu;
    df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return true;
  }
}

double smoothed01(const double &x, double &grad, const double mu) {
  static double mu4 = mu * mu * mu * mu;
  static double mu4_1 = 1.0 / mu4;
  if (x < -mu) {
    grad = 0;
    return 0;
  } else if (x < 0) {
    double y = x + mu;
    double y2 = y * y;
    grad = y2 * (mu - 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu - x) * mu4_1;
  } else if (x < mu) {
    double y = x - mu;
    double y2 = y * y;
    grad = y2 * (mu + 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
  } else {
    grad = 0;
    return 1;
  }
}

double penF(const double &x, double &grad) {
  static double eps = 0.05;
  static double eps2 = eps * eps;
  static double eps3 = eps * eps2;
  if (x < 2 * eps) {
    double x2 = x * x;
    double x3 = x * x2;
    double x4 = x2 * x2;
    grad = 12 / eps2 * x2 - 4 / eps3 * x3;
    return 4 / eps2 * x3 - x4 / eps3;
  } else {
    grad = 16;
    return 16 * (x - eps);
  }
}

double penF2(const double &x, double &grad) {
  double x2 = x * x;
  grad = 3 * x2;
  return x * x2;
}

/*
 * function getJacobian
 * for a quaternion rotation: r = q*p*q^(-1)
 * returns dr_dq
 */
void TrajOpt::getJacobian(const Eigen::Vector3d &p, const Eigen::Quaterniond &q,
                          Eigen::MatrixXd &Jacobian) {
  Jacobian.resize(4, 3);
  Jacobian.row(0) << p(0) * q.w() + p(2) * q.y() - p(1) * q.z(),
      p(1) * q.w() + p(0) * q.z() - p(2) * q.x(),
      p(1) * q.x() - p(0) * q.y() + p(2) * q.w();
  Jacobian.row(1) << p(0) * q.x() + p(1) * q.y() + p(2) * q.z(),
      p(0) * q.y() - p(1) * q.x() - p(2) * q.w(),
      p(1) * q.w() + p(0) * q.z() - p(2) * q.x();
  Jacobian.row(2) << p(1) * q.x() - p(0) * q.y() + p(2) * q.w(),
      p(0) * q.x() + p(1) * q.y() + p(2) * q.z(),
      p(1) * q.z() - p(0) * q.w() - p(2) * q.y();
  Jacobian.row(3) << p(2) * q.x() - p(0) * q.z() - p(1) * q.w(),
      p(0) * q.w() - p(1) * q.z() + p(2) * q.y(),
      p(0) * q.x() + p(1) * q.y() + p(2) * q.z();
  Jacobian = 2 * Jacobian;
}

Eigen::MatrixXd TrajOpt::cal_timebase_acc(const int order, const double &t) {
  double s1, s2, s3;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  Eigen::Matrix<double, 4, 1> beta;
  switch (order) {
  case 0:
    beta << 1.0, s1, s2, s3;
    break;
  case 1:
    beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
    break;
  case 2:
    beta << 0.0, 0.0, 2.0, 6.0 * s1;
    break;
  default:
    std::cout << "[trajopt] cal_timebase error." << std::endl;
    break;
  }
  return beta;
}

Eigen::MatrixXd TrajOpt::cal_timebase_jerk(const int order, const double &t) {
  double s1, s2, s3, s4, s5;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  s4 = s2 * s2;
  s5 = s4 * s1;
  Eigen::Matrix<double, 6, 1> beta;
  switch (order) {
  case 0:
    beta << 1.0, s1, s2, s3, s4, s5;
    break;
  case 1:
    beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
    break;
  case 2:
    beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
    break;
  case 3:
    beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
    break;
  default:
    std::cout << "[trajopt] cal_timebase error." << std::endl;
    break;
  }
  return beta;
}

Eigen::MatrixXd TrajOpt::cal_timebase_snap(const int order, const double &t) {
  double s1, s2, s3, s4, s5, s6, s7;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  s4 = s2 * s2;
  s5 = s4 * s1;
  s6 = s4 * s2;
  s7 = s4 * s3;
  Eigen::Matrix<double, 8, 1> beta;
  switch (order) {
  case 0:
    beta << 1.0, s1, s2, s3, s4, s5, s6, s7;
    break;
  case 1:
    beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5,
        7.0 * s6;
    break;
  case 2:
    beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    break;
  case 3:
    beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
    break;
  case 4:
    beta << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
    break;
  default:
    std::cout << "[trajopt] cal_timebase error." << std::endl;
    break;
  }
  return beta;
}

void forwardT_sum(const Eigen::Ref<const Eigen::VectorXd> &t, const double &sT,
                  Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();
  vecT(M) = 1.0 - vecT.sum();
  vecT *= sT;
  return;
}
void backwardT_sum(const Eigen::Ref<const Eigen::VectorXd> &vecT,
                   Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  t = vecT.head(M) / vecT(M);
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
void forwardT(const Eigen::Ref<const Eigen::VectorXd> &t,
              Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}
void backwardT(const Eigen::Ref<const Eigen::VectorXd> &vecT,
               Eigen::Ref<Eigen::VectorXd> t) {
  int M = vecT.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
void forwardT(const double &t, double &T) { T = expC2(t); }
void backwardT(const double &T, double &t) { t = logC2(T); }

double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd> &t,
                   const Eigen::Ref<const Eigen::VectorXd> &gradT,
                   Eigen::Ref<Eigen::VectorXd> gradt) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    gradt(i) = gradT(i) * gdT2t(t(i));
  }
  return;
}

void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd> &t, const double &sT,
                   const Eigen::Ref<const Eigen::VectorXd> &gradT,
                   Eigen::Ref<Eigen::VectorXd> gradt) {
  int Ms1 = t.size();
  Eigen::VectorXd gFree = sT * gradT.head(Ms1);
  double gTail = sT * gradT(Ms1);
  Eigen::VectorXd dExpTau(Ms1);
  double expTauSum = 0.0, gFreeDotExpTau = 0.0;
  double denSqrt, expTau;
  for (int i = 0; i < Ms1; i++) {
    if (t(i) > 0) {
      expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
      dExpTau(i) = t(i) + 1.0;
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    } else {
      denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
      expTau = 1.0 / denSqrt;
      dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    }
  }
  denSqrt = expTauSum + 1.0;
  gradt = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
          (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() /
              (denSqrt * denSqrt);
}

void TrajOpt::get_init_taj(const int &N, const Eigen::MatrixXd &init_P_vec,
                           Eigen::VectorXd &init_T_vec) {
  minco_s3_opt_.reset(initS_.block<3, 3>(0, 0), finalS_.block<3, 3>(0, 0), N);
  double desire_vel = vmax_;
  double max_vel, max_acc;
  int cnt = 0;
  do {
    INFO_MSG("1111");
    for (size_t i = 0; i < N; i++) {
      if (i == 0) {
        init_T_vec(i) = (init_P_vec.col(i) - initS_.col(0)).norm() / desire_vel;
      } else if (i == N - 1) {
        init_T_vec(i) =
            (finalS_.col(0) - init_P_vec.col(i - 1)).norm() / desire_vel;
      } else {
        init_T_vec(i) =
            (init_P_vec.col(i) - init_P_vec.col(i - 1)).norm() / desire_vel;
      }
    }
    INFO_MSG("desire_vel: " << desire_vel);
    INFO_MSG("init_T_vec: " << init_T_vec.transpose());

    minco_s3_opt_.generate(init_P_vec, init_T_vec);
    desire_vel /= 1.5;
    INFO_MSG("generate done");
    max_vel = minco_s3_opt_.getTraj().getMaxVelRate();
    max_acc = minco_s3_opt_.getTraj().getMaxAccRate();
    cnt++;
    INFO_MSG("max_vel: " << max_vel << ", max_acc: " << max_acc);

  } while (cnt < 3 && (max_vel > vmax_ || max_acc > amax_));

  // Trajectory<5> init_traj = minco_s3_opt_.getTraj();
  // visPtr_->visualize_traj(init_traj, "traj");
}

void TrajOpt::get_init_s4_taj(const int &N, const Eigen::MatrixXd &init_P_vec,
                              Eigen::VectorXd &init_T_vec) {
  minco_s4_opt_.reset(initS_.block<3, 4>(0, 0), finalS_.block<3, 4>(0, 0), N);
  double desire_vel = vmax_;
  double max_vel, max_acc;
  int cnt = 0;
  do {
    // INFO_MSG("1111");
    for (size_t i = 0; i < N; i++) {
      if (i == 0) {
        init_T_vec(i) = (init_P_vec.col(i) - initS_.col(0)).norm() / desire_vel;
      } else if (i == N - 1) {
        init_T_vec(i) =
            (finalS_.col(0) - init_P_vec.col(i - 1)).norm() / desire_vel;
      } else {
        init_T_vec(i) =
            (init_P_vec.col(i) - init_P_vec.col(i - 1)).norm() / desire_vel;
      }
    }
    // INFO_MSG("desire_vel: " << desire_vel);
    // INFO_MSG("init_T_vec: " << init_T_vec.transpose());

    minco_s4_opt_.generate(init_P_vec, init_T_vec);
    desire_vel /= 1.5;
    // INFO_MSG("generate done");
    max_vel = minco_s4_opt_.getTraj().getMaxVelRate();
    max_acc = minco_s4_opt_.getTraj().getMaxAccRate();
    cnt++;
    // INFO_MSG("max_vel: " << max_vel<<", max_acc: " << max_acc);

  } while (cnt < 3 && (max_vel > vmax_ || max_acc > amax_));

  // Trajectory<5> init_traj = minco_s4_opt_.getTraj();
  // visPtr_->visualize_traj(init_traj, "traj");
}

// bool TrajOpt::grad_cost_collision(const Eigen::Vector3d& p,
//                                   Eigen::Vector3d& gradp,
//                                   double& costp){
//   costp = 0.0;
//   gradp.setZero();
//   double dist_threshold = 0.2;
//   double          dist = 0;
//   Eigen::Vector3d dist_grad;
//   dist_grad.setZero();
//   dist = gridmapPtr_->getCostWithGrad(p, dist_grad);
//   if (dist < dist_threshold){
//     double pen =  dist_threshold - dist;
//     costp += pen * pen;

//     gradp += -2.0 * pen * dist_grad;
//     // INFO_MSG("p: "<<p.transpose()<<", esdf: "<< dist <<", pen: "<<pen<<",
//     costp: "<<costp<<", grad: "<<gradp.transpose());
//   }

//   gradp *= rhoP_;
//   costp *= rhoP_;
//   return true;
// }

bool TrajOpt::grad_cost_collision_rc(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &obs, // xyz or xyz r a
    Eigen::Vector3d &gradp, double &costp) {
  costp = 0.0;
  gradp.setZero();
  double dist = 0;
  Eigen::Vector3d dist_grad;
  dist_grad.setZero();

  Eigen::Vector3d dir = p - obs.head(3);
  dist = dir.norm();
  dist_grad = dir.normalized();

  if (dist < dist_threshold_) {
    double pen = dist_threshold_ - dist;
    costp += pen * pen;

    gradp += -2.0 * pen * dist_grad;
    // INFO_MSG("p: "<<p.transpose()<<", esdf: "<< dist <<", pen: "<<pen<<",
    // costp: "<<costp<<", grad: "<<gradp.transpose());
  } else
    return false;

  gradp *= rhoP_;
  costp *= rhoP_;

  return true;
}

bool TrajOpt::grad_cost_sphere_collision(
    const Eigen::Vector3d &p,
    const std::vector<Eigen::Vector3d> &obs_pts,
    const double &safety_distance, const double &sphere_radius,
    Eigen::Vector3d &gradp, double &costp) {
  gradp.setZero();
  costp = 0.0;
  if (obs_pts.empty()) {
    return false;
  }

  const double center_clearance =
      sphere_radius + safety_distance + irl_obs_optimization_margin_;
  const double mu = 0.01;
  double min_dist = std::numeric_limits<double>::infinity();
  Eigen::Vector3d nearest_dir = Eigen::Vector3d::Zero();
  Eigen::Vector3d obs_center = Eigen::Vector3d::Zero();
  for (const Eigen::Vector3d &obs : obs_pts) {
    obs_center += obs;
  }
  obs_center /= static_cast<double>(obs_pts.size());

  for (const Eigen::Vector3d &obs : obs_pts) {
    Eigen::Vector3d dir = p - obs;
    const double dist = dir.norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_dir = dir;
    }
  }

  if (!std::isfinite(min_dist)) {
    return false;
  }

  double grad_pen = 0.0;
  if (!smoothedL1(center_clearance - min_dist, mu, costp, grad_pen)) {
    return false;
  }

  const double inner_fallback_threshold = std::min(0.15, 0.5 * center_clearance);
  if (min_dist < inner_fallback_threshold) {
    Eigen::Vector3d radial_dir = p - obs_center;
    radial_dir.z() = 0.0;
    if (radial_dir.norm() > 1e-4) {
      nearest_dir = radial_dir;
    } else if (nearest_dir.norm() < 1e-4) {
      nearest_dir = Eigen::Vector3d::UnitY();
    }
  }

  gradp = -grad_pen * nearest_dir.normalized();
  gradp *= rhoP_;
  costp *= rhoP_;
  gradp *= irl_collision_weight_scale_;
  costp *= irl_collision_weight_scale_;
  return true;
}

bool TrajOpt::grad_cost_analytic_cylinder_collision(
    const Eigen::Vector3d &p, const Eigen::Vector2d &center_xy,
    const double &cylinder_radius, const double &z_min, const double &z_max,
    const double &safety_distance, const double &sphere_radius,
    Eigen::Vector3d &gradp, double &costp) {
  gradp.setZero();
  costp = 0.0;

  double signed_distance = 0.0;
  Eigen::Vector3d dist_grad = Eigen::Vector3d::Zero();
  if (!cylinderSignedDistanceAndGrad(p, center_xy, cylinder_radius, z_min,
                                     z_max, signed_distance, &dist_grad)) {
    return false;
  }

  const double required_clearance =
      sphere_radius + safety_distance + irl_obs_optimization_margin_;
  const double mu = 0.01;
  double grad_pen = 0.0;
  if (!smoothedL1(required_clearance - signed_distance, mu, costp, grad_pen)) {
    return false;
  }

  gradp = -grad_pen * dist_grad;
  gradp *= rhoP_;
  costp *= rhoP_;
  gradp *= irl_collision_weight_scale_;
  costp *= irl_collision_weight_scale_;
  return true;
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d &v, Eigen::Vector3d &gradv,
                          double &costv) {
  gradv.setZero();
  costv = 0;
  double vpen = v.squaredNorm() - vmax_ * vmax_;

  if (vpen > 0) {
    // INFO_MSG("v: " << v);
    // INFO_MSG("vmax: " << vmax_);
    double grad = 0;
    costv = smoothedL1(vpen, grad);
    gradv = rhoV_ * grad * 2 * v;
    costv *= rhoV_;
    return true;
  }
  return false;

  // double vpen = v.squaredNorm() - vmax_ * vmax_;
  // if (vpen > 0) {
  //   gradv = rhoV_ * 6 * vpen * vpen * v;
  //   costv = rhoV_ * vpen * vpen * vpen;
  //   return true;
  // }
  // return false;
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d &v, const double &v_max,
                          Eigen::Vector3d &gradv, double &costv) {
  gradv.setZero();
  costv = 0;
  double vpen = v.squaredNorm() - v_max * v_max;

  if (vpen > 0) {
    double grad = 0;
    costv = smoothedL1(vpen, grad);
    gradv = rhoV_ * grad * 2 * v;
    costv *= rhoV_;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d &v, const double &v_max,
                          const double &rhoV_weight, Eigen::Vector3d &gradv,
                          double &costv) {
  gradv.setZero();
  costv = 0;
  double vpen = v.squaredNorm() - v_max * v_max;

  if (vpen > 0) {
    double grad = 0;
    costv = smoothedL1(vpen, grad);
    gradv = rhoV_weight * grad * 2 * v;
    costv *= rhoV_weight;
    return true;
  }
  return false;
}

// bool TrajOpt::grad_cost_v(const Eigen::Vector3d &v, const double &v_max_horiz,
//                           const double &v_max_vert, Eigen::Vector3d &gradv,
//                           double &costv) {
//   gradv.setZero();
//   costv = 0;
//   bool ret = false;
//   double v_max_horiz_use = std::min(vmax_, v_max_horiz);
//   double vpen = v.head(2).squaredNorm() - v_max_horiz_use * v_max_horiz_use;
//   if (vpen > 0) {
//     // double grad = 0;
//     // costv += smoothedL1(vpen, grad);
//     // gradv.head(2) = grad * 2 * v.head(2);

//     costv += vpen * vpen * vpen;
//     gradv.head(2) = 6 * vpen * vpen * v.head(2);

//     ret = true;
//   }

//   vpen = v.z() * v.z() - v_max_vert * v_max_vert;
//   if (vpen > 0) {
//     // double grad = 0;
//     // costv += smoothedL1(vpen, grad);
//     // gradv.z() = grad * 2 * v.z();

//     costv += vpen * vpen * vpen;
//     gradv.z() = 6 * vpen * vpen * v.z();

//     ret = true;
//   }

//   costv *= rhoV_;
//   gradv *= rhoV_;

//   return ret;
// }

bool TrajOpt::grad_cost_a(const Eigen::Vector3d &a, Eigen::Vector3d &grada,
                          double &costa) {
  grada.setZero();
  costa = 0;
  double apen = a.squaredNorm() - amax_ * amax_;

  if (apen > 0) {
    // INFO_MSG("a: " << a);
    // INFO_MSG("amax: " << amax_);
    double grad = 0;
    costa = smoothedL1(apen, grad);
    grada = rhoA_ * grad * 2 * a;
    costa *= rhoA_;
    return true;
  }
  return false;

  // double apen = a.squaredNorm() - amax_ * amax_;
  // if (apen > 0) {
  //   grada = rhoA_ * 6 * apen * apen * a;
  //   costa = rhoA_ * apen * apen * apen;
  //   return true;
  // }
  // return false;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d &a, const double &a_max,
                          Eigen::Vector3d &grada, double &costa) {
  grada.setZero();
  costa = 0;
  double apen = a.squaredNorm() - a_max * a_max;

  if (apen > 0) {
    double grad = 0;
    costa = smoothedL1(apen, grad);
    grada = rhoA_ * grad * 2 * a;
    costa *= rhoA_;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d &a, const double &a_max,
                          const double &rhoA_weight, Eigen::Vector3d &grada,
                          double &costa) {
  grada.setZero();
  costa = 0;
  double apen = a.squaredNorm() - a_max * a_max;

  if (apen > 0) {
    double grad = 0;
    costa = smoothedL1(apen, grad);
    grada = rhoA_weight * grad * 2 * a;
    costa *= rhoA_weight;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_dyaw(const double &dyaw, double &grad_dyaw,
                             double &cost_dyaw) {
  grad_dyaw = 0;
  cost_dyaw = 0;
  double vpen = dyaw * dyaw - dyaw_max_ * dyaw_max_;

  if (vpen > 0) {
    // INFO_MSG("v: " << v);
    // INFO_MSG("vmax: " << vmax_);
    double grad = 0;
    cost_dyaw = smoothedL1(vpen, grad);
    grad_dyaw = rhoDyaw_ * grad * 2 * dyaw;
    cost_dyaw *= rhoDyaw_;
    // if (grad_dyaw != 0) INFO_MSG_RED("grad_dyaw: "<< grad_dyaw);
    return true;
  }
  return false;
}

// Theta (arm joint angle) position constraints
bool TrajOpt::grad_cost_theta(const Eigen::VectorXd &thetas,
                              Eigen::VectorXd &grad_thetas,
                              double &cost_thetas) {
  grad_thetas.setZero();
  cost_thetas = 0;
  bool has_violation = false;

  for (int i = 0; i < thetas.size(); ++i) {
    // Check lower bound violation
    double pen_min = theta_min_(i) - thetas(i);
    if (pen_min > 0) {
      double grad = 0;
      double cost = smoothedL1(pen_min, grad);
      grad_thetas(i) += -rhoTheta_ * grad;
      cost_thetas += rhoTheta_ * cost;
      has_violation = true;
    }

    // Check upper bound violation
    double pen_max = thetas(i) - theta_max_(i);
    if (pen_max > 0) {
      double grad = 0;
      double cost = smoothedL1(pen_max, grad);
      grad_thetas(i) += rhoTheta_ * grad;
      cost_thetas += rhoTheta_ * cost;
      has_violation = true;
    }
  }

  return has_violation;
}

// Theta velocity constraints
bool TrajOpt::grad_cost_dtheta(const Eigen::VectorXd &dthetas,
                               Eigen::VectorXd &grad_dthetas,
                               double &cost_dthetas) {
  grad_dthetas.setZero();
  cost_dthetas = 0;
  bool has_violation = false;

  for (int i = 0; i < dthetas.size(); ++i) {
    double pen = dthetas(i) * dthetas(i) - dtheta_max_(i) * dtheta_max_(i);

    if (pen > 0) {
      double grad = 0;
      double cost = smoothedL1(pen, grad);
      grad_dthetas(i) = rhoDtheta_ * grad * 2 * dthetas(i);
      cost_dthetas += rhoDtheta_ * cost;
      has_violation = true;
    }
  }

  return has_violation;
}

// bool TrajOpt::grad_cost_yaw(const double& yaw,
//                             const Eigen::Vector3d& p,
//                             const Eigen::Vector3d& target_p,
//                             double& grad_yaw,
//                             double& cost_yaw){
//   grad_yaw = 0;
//   cost_yaw = 0;
//   Eigen::Vector3d dp = target_p - p;
//   double exp_yaw = atan2(dp.y(), dp.x());
//   double pen = -rot_util::error_angle(yaw, exp_yaw);

//   // INFO_MSG_GREEN("p: "<<p.transpose()<<", tar: "<<target_p.transpose());
//   std::string sgn;
//   cost_yaw = rhoYaw_ * pen * pen;
//   grad_yaw = rhoYaw_ * 2 * pen;
//   if (grad_yaw >= 0){
//     sgn = "-";
//   }else{
//     sgn = "+";
//   }
//   // INFO_MSG("("<<sgn << ")yaw: " << yaw << ", exp_yaw: " << exp_yaw << ",
//   error_angle: " << pen);

//   return true;
// }

// bool TrajOpt::grad_cost_yaw(const double& yaw,
//                             const Eigen::Vector3d& vel,
//                             Eigen::Vector3d& grad_vel,
//                             double& grad_yaw,
//                             double& cost_yaw){
//   grad_yaw = 0;
//   cost_yaw = 0;
//   double exp_yaw;
//   double yaw_;
//   // if (abs(vel(0)) <= 0.1 && abs(vel(1)) <= 0.1)
//   if (vel(0)==0 && vel(1)==0)
//   {
//     return false;
//   }
//   else
//   {
//     exp_yaw = atan2(vel(1), vel(0));
//     yaw_ = yaw;
//     // wrap angle
//     if (yaw_ < -M_PI) yaw_ = yaw_ + 2 * M_PI;
//     else if (yaw_ > M_PI)  yaw_ = yaw_ - 2 * M_PI;

//     double pen = -rot_util::error_angle(yaw_, exp_yaw);

//     std::string sgn;
//     cost_yaw = rhoYaw_ * pen * pen;
//     grad_yaw = rhoYaw_ * 2 * pen;
//     grad_vel << -rhoYaw_ * 2 * pen * (-vel(1)/ ((vel(0)*vel(0)) +
//     (vel(1)*vel(1)))), -rhoYaw_ * 2 * pen * (vel(0)/ ((vel(0)*vel(0)) +
//     (vel(1)*vel(1)))), 0;

//     if (grad_yaw >= 0){
//       sgn = "-";
//     }else{
//       sgn = "+";
//     }
//     INFO_MSG_YELLOW("grad vel: " << grad_vel.transpose());
//     INFO_MSG("("<<sgn << ")yaw: " << yaw << ", exp_yaw: " << exp_yaw << ",
//     error_angle: " << pen << ", grad_yaw: " << grad_yaw);
//   }

//   return true;
// }

bool TrajOpt::grad_cost_yaw_forward(const double &yaw,
                                    const Eigen::Vector3d &vel,
                                    double &grad_yaw, Eigen::Vector3d &grad_vel,
                                    double &cost_yaw) {
  grad_yaw = 0;
  cost_yaw = 0;
  grad_vel.setZero();

  if (vel.head(2).norm() >= 5e-1) {
    Eigen::Vector3d dYawdVel;
    dYawdVel.setZero();
    // type1
    double exp_yaw = atan2(vel(1), vel(0));
    double error_angle = rot_util::error_angle(yaw, exp_yaw);
    double tol = 10.0 / 180.0 * M_PI; // tolerance
    double pen = error_angle * error_angle - tol * tol;
    if (pen > 0) {
      double df = 0;
      cost_yaw += rhoYaw_ * smoothedL1(pen, df);
      grad_yaw += -rhoYaw_ * df * 2.0 * error_angle;
      Eigen::Vector3d dYawdVel(-vel(1), vel(0), 0);
      dYawdVel = dYawdVel / vel.head(2).squaredNorm();
      grad_vel += rhoYaw_ * df * 2.0 * error_angle * dYawdVel;

      // check grad
      // exp_yaw = yaw + error_angle;
      // INFO_MSG_RED("yaw: " << yaw << ", exp_yaw: " << exp_yaw << ",
      // error_angle: " << error_angle); double disturb = 1e-6; Eigen::Vector3d
      // vel_disturb = vel; double yaw_disturb = yaw; yaw_disturb += disturb;
      // double exp_yaw_disturb = atan2(vel_disturb(1), vel_disturb(0));
      // double error_angle_disturb = rot_util::error_angle(yaw_disturb,
      // exp_yaw_disturb); double pen_disturb = error_angle_disturb *
      // error_angle_disturb - tol * tol; double df_disturb = 0; double
      // cost_yaw_disturb = rhoYaw_ * smoothedL1(pen_disturb, df_disturb);
      // double grad_yaw_disturb = (cost_yaw_disturb - cost_yaw) / disturb;
      // std::cout << "grad_yaw: " << grad_yaw << ", grad_yaw_disturb: " <<
      // grad_yaw_disturb << std::endl;

      // Eigen::Vector2d grad_vel_disturb;
      // yaw_disturb = yaw;
      // vel_disturb(0) += disturb;
      // exp_yaw_disturb = atan2(vel_disturb(1), vel_disturb(0));
      // error_angle_disturb = rot_util::error_angle(yaw_disturb,
      // exp_yaw_disturb); pen_disturb = error_angle_disturb *
      // error_angle_disturb - tol * tol; df_disturb = 0; cost_yaw_disturb =
      // rhoYaw_ * smoothedL1(pen_disturb, df_disturb); grad_vel_disturb(0) =
      // (cost_yaw_disturb - cost_yaw) / disturb;

      // vel_disturb = vel;
      // vel_disturb(1) += disturb;
      // exp_yaw_disturb = atan2(vel_disturb(1), vel_disturb(0));
      // error_angle_disturb = rot_util::error_angle(yaw_disturb,
      // exp_yaw_disturb); pen_disturb = error_angle_disturb *
      // error_angle_disturb - tol * tol; df_disturb = 0; cost_yaw_disturb =
      // rhoYaw_ * smoothedL1(pen_disturb, df_disturb); grad_vel_disturb(1) =
      // (cost_yaw_disturb - cost_yaw) / disturb; std::cout << "grad_vel: " <<
      // grad_vel.head(2).transpose() << ", grad_yaw_disturb: " <<
      // grad_vel_disturb.transpose() << std::endl;

      return true;
    }

    // type 2
    // Eigen::Vector2d dir_yaw(cos(yaw), sin(yaw));
    // Eigen::Vector2d dir_exp_yaw = vel.head(2).normalized();
    // double pen = 1.0 - dir_yaw.dot(dir_exp_yaw) - 0.1; // 0.1 is the
    // tolerance if(pen > 0){
    //   double df = 0;
    //   cost_yaw = rhoYaw_ * smoothedL1(pen, df);
    //   grad_yaw = -rhoYaw_ * df * dir_exp_yaw.transpose() *
    //   Eigen::Vector2d(-sin(yaw), cos(yaw)); grad_vel.head(2) = -rhoYaw_ * df
    //   * dir_yaw.transpose() * (Eigen::Matrix2d::Identity() - dir_exp_yaw *
    //   dir_exp_yaw.transpose()) / vel.head(2).norm(); return true;
    // }
  }
  return false;
}

bool TrajOpt::grad_cost_yaw(const double &yaw, const Eigen::Vector3d &p,
                            const Eigen::Vector3d &target_p, double &grad_yaw,
                            double &cost_yaw) {
  grad_yaw = 0;
  cost_yaw = 0;
  Eigen::Vector3d dp = target_p - p;
  double exp_yaw = atan2(dp.y(), dp.x());
  double pen = -rot_util::error_angle(yaw, exp_yaw);

  // INFO_MSG_GREEN("p: "<<p.transpose()<<", tar: "<<target_p.transpose());
  std::string sgn;
  cost_yaw = rhoYaw_ * pen * pen;
  grad_yaw = rhoYaw_ * 2 * pen;
  if (grad_yaw >= 0) {
    sgn = "-";
  } else {
    sgn = "+";
  }
  // INFO_MSG("("<<sgn << ")yaw: " << yaw << ", exp_yaw: " << exp_yaw << ",
  // error_angle: " << pen);

  return true;
}

Trajectory<5> getS3TrajWithYaw(minco::MINCO_S3 &mincos3_opt,
                               minco::MINCO_S2 &mincoyaw_opt) {
  Trajectory<5> traj;
  traj.clear();
  traj.reserve(mincos3_opt.N);
  for (int i = 0; i < mincos3_opt.N; i++) {
    traj.emplace_back(
        mincos3_opt.T1(i),
        mincos3_opt.b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse(),
        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}

Trajectory<7> getS4UTrajWithYaw(minco::MINCO_S4_Uniform &mincos4u_opt,
                                minco::MINCO_S2 &mincoyaw_opt) {
  Trajectory<7> traj;
  traj.clear();
  traj.reserve(mincos4u_opt.N);
  for (int i = 0; i < mincos4u_opt.N; i++) {
    traj.emplace_back(
        mincos4u_opt.t(1),
        mincos4u_opt.c.block<8, 3>(8 * i, 0).transpose().rowwise().reverse(),
        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}

Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4 &mincos4_opt,
                               minco::MINCO_S2 &mincoyaw_opt) {
  Trajectory<7> traj;
  traj.clear();
  traj.reserve(mincos4_opt.N);
  for (int i = 0; i < mincos4_opt.N; i++) {
    traj.emplace_back(
        mincos4_opt.T1(i),
        mincos4_opt.b.block<8, 3>(8 * i, 0).transpose().rowwise().reverse(),
        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}

Trajectory<7>
getS4TrajWithYawAndThetas(minco::MINCO_S4 &mincos4_opt,
                          minco::MINCO_S2 &mincoyaw_opt,
                          std::vector<minco::MINCO_S2> &minco_thetas_opt_vec) {
  Trajectory<7> traj;
  traj.clear();
  traj.reserve(mincos4_opt.N);

  std::vector<AngleCoefficientMat> thetas_coef_vec;
  // for(size_t i = 0; i < thetas_coef_vec.size(); i++)
  //   thetas_coef_vec.emplace_back(minco_thetas_opt_vec[i].b.block<4, 1>(4 * i,
  //   0).transpose().rowwise().reverse());

  for (int i = 0; i < mincos4_opt.N; i++) {
    thetas_coef_vec.clear();
    for (size_t j = 0; j < minco_thetas_opt_vec.size(); j++)
      thetas_coef_vec.emplace_back(minco_thetas_opt_vec[j]
                                       .b.block<4, 1>(4 * i, 0)
                                       .transpose()
                                       .rowwise()
                                       .reverse());

    traj.emplace_back(
        mincos4_opt.T1(i),
        mincos4_opt.b.block<8, 3>(8 * i, 0).transpose().rowwise().reverse(),
        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse(),
        thetas_coef_vec);
  }
  return traj;
}
} // namespace traj_opt
