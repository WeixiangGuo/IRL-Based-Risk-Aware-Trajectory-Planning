#include <traj_opt/lbfgs_raw.hpp>
#include <traj_opt/traj_opt.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace traj_opt {
using rot_util = rotation_util::RotUtil;

static double tictoc_innerloop_;
static double tictoc_integral_;

namespace {

struct IrlMincoDebugMetrics {
  double path_len = 0.0;
  double chord_len = 0.0;
  double length_ratio = 1.0;
  double min_clearance_1 = std::numeric_limits<double>::infinity();
  double min_clearance_2 = std::numeric_limits<double>::infinity();
  double min_margin_1 = std::numeric_limits<double>::infinity();
  double min_margin_2 = std::numeric_limits<double>::infinity();
  int worst_group = 0;
  Eigen::Vector3d worst_pos = Eigen::Vector3d::Zero();
  int reverse_count = 0;
  int self_cross_count = 0;
  bool negative_margin = false;
  bool possible_loop = false;
};

double debugCylinderSignedDistance(const Eigen::Vector3d &p,
                                   const Eigen::Vector2d &center_xy,
                                   const double cylinder_radius,
                                   const double z_min,
                                   const double z_max) {
  const double radius = std::max(cylinder_radius, 1e-6);
  const double z_low = std::min(z_min, z_max);
  const double z_high = std::max(z_min, z_max);
  const Eigen::Vector2d dxy = p.head<2>() - center_xy;
  const double side = dxy.norm() - radius;
  const double below = z_low - p.z();
  const double above = p.z() - z_high;

  if (side > 0.0 || below > 0.0 || above > 0.0) {
    const double radial_excess = std::max(side, 0.0);
    const double z_excess = std::max(std::max(below, above), 0.0);
    return std::sqrt(radial_excess * radial_excess + z_excess * z_excess);
  }

  const double inside_side = side;
  const double inside_bottom = p.z() - z_low;
  const double inside_top = z_high - p.z();
  return std::max(inside_side, -std::min(inside_bottom, inside_top));
}

double debugMinDistanceToPoints(const Eigen::Vector3d &p,
                                const std::vector<Eigen::Vector3d> &points) {
  double min_sq = std::numeric_limits<double>::infinity();
  for (const auto &obs : points) {
    min_sq = std::min(min_sq, (p - obs).squaredNorm());
  }
  return std::isfinite(min_sq) ? std::sqrt(min_sq)
                               : std::numeric_limits<double>::infinity();
}

double orient2d(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                const Eigen::Vector3d &c) {
  return (b.x() - a.x()) * (c.y() - a.y()) -
         (b.y() - a.y()) * (c.x() - a.x());
}

bool bboxOverlap2d(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                   const Eigen::Vector3d &c, const Eigen::Vector3d &d) {
  constexpr double eps = 1e-6;
  return std::max(std::min(a.x(), b.x()), std::min(c.x(), d.x())) <=
             std::min(std::max(a.x(), b.x()), std::max(c.x(), d.x())) + eps &&
         std::max(std::min(a.y(), b.y()), std::min(c.y(), d.y())) <=
             std::min(std::max(a.y(), b.y()), std::max(c.y(), d.y())) + eps;
}

bool segmentsIntersect2d(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                         const Eigen::Vector3d &c, const Eigen::Vector3d &d) {
  if (!bboxOverlap2d(a, b, c, d)) {
    return false;
  }
  const double o1 = orient2d(a, b, c);
  const double o2 = orient2d(a, b, d);
  const double o3 = orient2d(c, d, a);
  const double o4 = orient2d(c, d, b);
  return o1 * o2 < -1e-8 && o3 * o4 < -1e-8;
}

void sampleDebugTrajectory(const Trajectory<7> &traj, const double sample_dt,
                           std::vector<Eigen::Vector3d> &points) {
  points.clear();
  const double dt = std::max(sample_dt, 0.01);
  int piece_idx = 0;
  for (const auto &piece : traj) {
    const double dur = piece.getDuration();
    int steps = std::max(1, static_cast<int>(std::ceil(dur / dt)));
    for (int i = 0; i <= steps; ++i) {
      if (piece_idx > 0 && i == 0) {
        continue;
      }
      const double t = dur * static_cast<double>(i) / static_cast<double>(steps);
      points.push_back(piece.getPos(t));
    }
    ++piece_idx;
  }
}

IrlMincoDebugMetrics computeIrlMincoDebugMetrics(
    const TrajOpt &obj, const std::vector<Eigen::Vector3d> &points) {
  IrlMincoDebugMetrics metrics;
  if (points.size() < 2) {
    return metrics;
  }

  metrics.chord_len = (points.back() - points.front()).norm();
  Eigen::Vector3d prev_seg = Eigen::Vector3d::Zero();
  for (size_t i = 1; i < points.size(); ++i) {
    const Eigen::Vector3d seg = points[i] - points[i - 1];
    const double seg_len = seg.norm();
    metrics.path_len += seg_len;
    if (seg_len > 1e-6 && prev_seg.norm() > 1e-6 &&
        seg.normalized().dot(prev_seg.normalized()) < -0.35) {
      metrics.reverse_count += 1;
    }
    if (seg_len > 1e-6) {
      prev_seg = seg;
    }
  }
  metrics.length_ratio =
      metrics.chord_len > 1e-6 ? metrics.path_len / metrics.chord_len : 1.0;

  for (size_t i = 1; i + 2 < points.size(); ++i) {
    for (size_t j = i + 3; j < points.size(); ++j) {
      if (segmentsIntersect2d(points[i - 1], points[i], points[j - 1],
                              points[j])) {
        metrics.self_cross_count += 1;
      }
    }
  }

  auto update_worst = [&](const Eigen::Vector3d &p, const int group,
                          const double clearance, const double margin) {
    if (group == 1 && clearance < metrics.min_clearance_1) {
      metrics.min_clearance_1 = clearance;
      metrics.min_margin_1 = margin;
    } else if (group == 2 && clearance < metrics.min_clearance_2) {
      metrics.min_clearance_2 = clearance;
      metrics.min_margin_2 = margin;
    }

    const double current_worst_margin =
        std::min(metrics.min_margin_1, metrics.min_margin_2);
    if (std::abs(margin - current_worst_margin) < 1e-9 ||
        margin < current_worst_margin) {
      metrics.worst_group = group;
      metrics.worst_pos = p;
    }
  };

  for (const auto &p : points) {
    if (obj.enable_irl_analytic_cylinder_collision_) {
      const double signed_distance_1 = debugCylinderSignedDistance(
          p, obj.irl_cylinder1_center_xy_, obj.irl_cylinder1_radius_,
          obj.irl_cylinder_z_min_, obj.irl_cylinder_z_max_);
      const double signed_distance_2 = debugCylinderSignedDistance(
          p, obj.irl_cylinder2_center_xy_, obj.irl_cylinder2_radius_,
          obj.irl_cylinder_z_min_, obj.irl_cylinder_z_max_);
      const double clearance_1 = signed_distance_1 - obj.irl_uam_sphere_radius_;
      const double clearance_2 = signed_distance_2 - obj.irl_uam_sphere_radius_;
      update_worst(p, 1, clearance_1,
                   clearance_1 - obj.irl_safety_distance_1_);
      update_worst(p, 2, clearance_2,
                   clearance_2 - obj.irl_safety_distance_2_);
    } else {
      const double clearance_1 =
          debugMinDistanceToPoints(p, obj.aabb_pts_group1_vec_) -
          obj.irl_uam_sphere_radius_;
      const double clearance_2 =
          debugMinDistanceToPoints(p, obj.aabb_pts_group2_vec_) -
          obj.irl_uam_sphere_radius_;
      if (std::isfinite(clearance_1)) {
        update_worst(p, 1, clearance_1,
                     clearance_1 - obj.irl_safety_distance_1_);
      }
      if (std::isfinite(clearance_2)) {
        update_worst(p, 2, clearance_2,
                     clearance_2 - obj.irl_safety_distance_2_);
      }
    }
  }

  metrics.negative_margin =
      std::min(metrics.min_margin_1, metrics.min_margin_2) < -1e-3;
  metrics.possible_loop = metrics.self_cross_count > 0 ||
                          metrics.reverse_count > 2 ||
                          metrics.length_ratio > 1.8;
  return metrics;
}

void setDebugColor(visualization_msgs::Marker &marker,
                   const std::string &stage, const bool force,
                   const bool warn) {
  if (warn) {
    marker.color.r = 1.0;
    marker.color.g = 0.05;
    marker.color.b = 0.02;
  } else if (force) {
    marker.color.r = 0.05;
    marker.color.g = 0.95;
    marker.color.b = 0.25;
  } else if (stage.find("stage1") != std::string::npos) {
    marker.color.r = 0.15;
    marker.color.g = 0.55;
    marker.color.b = 1.0;
  } else {
    marker.color.r = 1.0;
    marker.color.g = 0.58;
    marker.color.b = 0.08;
  }
  marker.color.a = force ? 0.95 : 0.48;
}

std::string finiteOrDash(const double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3) << value;
  return ss.str();
}

} // namespace

bool TrajOpt::warm_start_cache_matches(
    const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
    const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
    const Eigen::MatrixXd &init_thetas, const Eigen::MatrixXd &final_thetas,
    const std::vector<Eigen::Vector3d> &ego_path) const {
  if (!minco_warm_start_cache_enable_ || !minco_warm_start_cache_ready_) {
    return false;
  }
  if (cached_warm_start_N_ != N_ ||
      cached_warm_start_dim_t_ != dim_t_ ||
      cached_warm_start_dim_p_ != dim_p_ ||
      cached_warm_start_n_thetas_ != n_thetas_) {
    return false;
  }

  const int opt_dim = dim_t_ + (4 + n_thetas_) * dim_p_;
  if (static_cast<int>(cached_warm_start_x_.size()) != opt_dim) {
    return false;
  }
  if (minco_warm_start_cache_force_reuse_) {
    return true;
  }

  auto matrix_matches = [&](const Eigen::MatrixXd &lhs,
                            const Eigen::MatrixXd &rhs) {
    return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols() &&
           lhs.isApprox(rhs, minco_warm_start_cache_tol_);
  };
  if (!matrix_matches(cached_warm_start_initS_, iniState) ||
      !matrix_matches(cached_warm_start_finalS_, finState) ||
      !matrix_matches(cached_warm_start_init_yaw_, init_yaw) ||
      !matrix_matches(cached_warm_start_final_yaw_, final_yaw) ||
      !matrix_matches(cached_warm_start_init_thetas_, init_thetas) ||
      !matrix_matches(cached_warm_start_final_thetas_, final_thetas)) {
    return false;
  }

  if (cached_warm_start_path_.size() != ego_path.size()) {
    return false;
  }
  for (size_t i = 0; i < ego_path.size(); ++i) {
    if ((cached_warm_start_path_[i] - ego_path[i]).norm() >
        minco_warm_start_cache_tol_) {
      return false;
    }
  }
  return true;
}

void TrajOpt::store_warm_start_cache(
    const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
    const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
    const Eigen::MatrixXd &init_thetas, const Eigen::MatrixXd &final_thetas,
    const std::vector<Eigen::Vector3d> &ego_path, const int opt_dim) {
  if (!minco_warm_start_cache_enable_ || opt_dim <= 0 || x_ == nullptr) {
    return;
  }
  cached_warm_start_N_ = N_;
  cached_warm_start_dim_t_ = dim_t_;
  cached_warm_start_dim_p_ = dim_p_;
  cached_warm_start_n_thetas_ = n_thetas_;
  cached_warm_start_x_.assign(x_, x_ + opt_dim);
  cached_warm_start_initS_ = iniState;
  cached_warm_start_finalS_ = finState;
  cached_warm_start_init_yaw_ = init_yaw;
  cached_warm_start_final_yaw_ = final_yaw;
  cached_warm_start_init_thetas_ = init_thetas;
  cached_warm_start_final_thetas_ = final_thetas;
  cached_warm_start_path_ = ego_path;
  minco_warm_start_cache_ready_ = true;
  ROS_INFO("[TrajOpt] stored stage-1 warm-start cache: N=%d dim=%d path_pts=%zu",
           N_, opt_dim, cached_warm_start_path_.size());
}

// Generate MincoS3(none-uniform) trajectory for given goal
bool TrajOpt::generate_traj_clutter(
    const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
    const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
    const Eigen::MatrixXd &init_thetas, const Eigen::MatrixXd &final_thetas,
    const double &seg_per_dis, const std::vector<Eigen::Vector3d> &ego_path,
    Trajectory<7> &traj) {

  INFO_MSG_GREEN_BG("----- Start gen traj");

  // if ((iniState.col(0) - finState.col(0)).norm() < gridmapPtr_->resolution())
  if ((iniState.col(0) - finState.col(0)).norm() < 0.01) {
    ROS_INFO("reach goal, no need to generate traj");
    return false;
  }

  // get whole_states_end from p_end2w & R_end2w
  double yaw_out;
  Eigen::Vector3d pos_out;
  final_thetas_.resize(n_thetas_, 2);
  final_thetas_.setZero();
  std::cout << "init_thetas: " << init_thetas << std::endl;
  std::cout << "final_thetas: " << final_thetas << std::endl;
  final_thetas_ = final_thetas;

  // 区分每次迭代，方便调试
  clear_cost_rec();
  dashboard_cost_print();

  //! 1. init flatmap params
  Eigen::VectorXd physicalParams(6);
  // params from px4 control
  physicalParams(0) = 0.61;
  physicalParams(1) = 9.8;
  physicalParams(2) = 0.10;
  physicalParams(3) = 0.23;
  physicalParams(4) = 0.01;
  physicalParams(5) = 0.02;
  flatmap_.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                 physicalParams(3), physicalParams(4), physicalParams(5));

  INFO_MSG_GREEN_BG("[GenTraj] 1 Params set done");

  //! 2. determine number of segment (a-star path raycheck-aware)
  std::vector<Eigen::Vector3d> mid_q_vec;
  if (en_through_ring_){
    if (has_ring_pose_){
      append_ring_mid_points(iniState, mid_q_vec);
      append_tail_constraint_mid_points(iniState, finState, final_yaw,
                                        final_thetas, mid_q_vec, N_);
    }else{
      std::cout << "No ring pose!" << std::endl;
      return false;
    }
  }else if (en_tail_constraint_) {
    extract_mid_pts_from_apath(ego_path, seg_per_dis, mid_q_vec, N_);
    append_tail_constraint_mid_points(iniState, finState, final_yaw,
                                      final_thetas, mid_q_vec, N_);
  } else {
    extract_mid_pts_from_apath(ego_path, seg_per_dis, mid_q_vec, N_);
  }
  visPtr_->visualize_pointcloud(mid_q_vec, "mid_waypts");

  inter_wps_ = mid_q_vec;
  inter_wps_.push_back(ego_path.back());
  set_astar_tracking_path(ego_path);

  // N_ = ego_path.size() - 1;
  INFO_MSG("[trajopt] Pieces: " << N_);
  INFO_MSG_GREEN_BG("[GenTraj] 2 Extract mid points done");

  //! 3. set opt varibles
  dim_t_ = N_;
  dim_p_ = N_ - 1;
  const int opt_dim = dim_t_ + (4 + n_thetas_) * dim_p_;
  x_ = new double[opt_dim];
  Eigen::Map<Eigen::VectorXd> t(x_, dim_t_);
  Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_);
  Eigen::Map<Eigen::VectorXd> yaw(x_ + dim_t_ + 3 * dim_p_, dim_p_);
  std::vector<Eigen::Map<Eigen::VectorXd>> thetas_vec;
  for (int i = 0; i < n_thetas_; ++i)
    thetas_vec.push_back(
        Eigen::Map<Eigen::VectorXd>(x_ + dim_t_ + (4 + i) * dim_p_, dim_p_));

  //! 4. set boundary & initial value
  initS_ = iniState;
  finalS_ = finState;

  // get yaw init value
  init_yaw_.resize(1, 2);
  final_yaw_.resize(1, 2);
  init_yaw_ = init_yaw;
  final_yaw_ = final_yaw;
  double error_angle_yaw =
      rot_util::error_angle(init_yaw(0, 0), final_yaw_(0, 0));
  final_yaw_(0, 0) = init_yaw_(0, 0) + error_angle_yaw;
  std::cout << "yaw_in: " << init_yaw(0, 0) << ", yaw_out: " << final_yaw(0, 0)
            << std::endl;

  //* average
  if (en_tail_constraint_) {
    for (int i = 0; i < N_ - 1; ++i) {
      if (i >= N_ - 3)
        yaw(i) = final_yaw(0, 0);
      else
        yaw(i) = init_yaw(0, 0) +
                 ((double)(i + 1) / (double)(N_ - 1)) * error_angle_yaw;
    }
  } else {
    for (int i = 0; i < N_ - 1; ++i)
      yaw(i) =
          init_yaw(0, 0) + ((double)(i + 1) / (double)N_) * error_angle_yaw;
  }

  //* look forward
  // for(int i = 1; i < N_; ++i){
  //   Eigen::Vector2d vel_i = traj_info.getJuncVel(i).head(2);
  //   yaw(i - 1) = atan2(vel_i(1), vel_i(0));
  //   if(i > 1){
  //     error_angle_yaw = rot_util::error_angle(yaw(i - 2), yaw(i - 1));
  //     yaw(i - 1) = yaw(i - 2) + error_angle_yaw;
  //   }else{
  //     error_angle_yaw = rot_util::error_angle(init_yaw_(0,0), yaw(i - 1));
  //     yaw(i - 1) = init_yaw_(0,0) + error_angle_yaw;
  //   }
  // }
  // error_angle_yaw = rot_util::error_angle(yaw(N_ - 2), final_yaw_(0, 0));
  // final_yaw_(0, 0) = yaw(N_ - 2) + error_angle_yaw;
  // INFO_MSG_RED("yaw_in: " << init_yaw_(0,0)<<", yaw_out: " <<
  // final_yaw_(0,0));

  // thetas initial value
  init_thetas_.resize(n_thetas_, 2);
  init_thetas_ = init_thetas;
  Eigen::VectorXd error_angle_thetas(n_thetas_);
  for (int i = 0; i < n_thetas_; ++i) {
    error_angle_thetas(i) =
        rot_util::error_angle(init_thetas_(i, 0), final_thetas_(i, 0));
    final_thetas_(i, 0) = init_thetas_(i, 0) + error_angle_thetas(i);
    if (en_tail_constraint_) {
      std::cout << "i:" << i << ",N_:" << N_ << std::endl;
      for (int j = 0; j < N_ - 1; ++j) {
        if (j >= N_ - 3) {
          std::cout << "j1:" << j << std::endl;
          thetas_vec[i](j) = final_thetas_(i, 0);
        } else {
          thetas_vec[i](j) =
              init_thetas_(i, 0) +
              ((double)(j + 1) / (double)(N_ - 2)) * error_angle_thetas(i);
          std::cout << "j2:" << j << std::endl;
        }
      }
      std::cout << "thetas_vec(" << i << "): " << thetas_vec[i].transpose()
                << std::endl;
    } else {
      for (int j = 0; j < N_ - 1; ++j)
        thetas_vec[i](j) =
            init_thetas_(i, 0) +
            ((double)(j + 1) / (double)(N_)) * error_angle_thetas(i);
    }
    minco_s2_theta_opt_vec_[i].reset(init_thetas_.block(i, 0, 1, 2),
                                     final_thetas_.block(i, 0, 1, 2), N_);
  }

  // pre-compute goal end-effector position for collision threshold switching
  {
    Eigen::Quaterniond q_goal(
        Eigen::AngleAxisd(final_yaw_(0, 0), Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d p_goal = finState.col(0);
    end_pos_goal_ = p_goal;
    if (rc_sdf_ptr_) {
      Eigen::Quaterniond q_goal_end;
      Eigen::Matrix4d T_goal_end;
      rc_sdf_ptr_->kine_ptr_->getEndPose(p_goal, q_goal, final_thetas_.col(0),
                                         end_pos_goal_, q_goal_end,
                                         T_goal_end);
    }
  }
  // print initial value
  std::cout << "yaw_in:" << init_yaw(0, 0) << ", yaw_out:" << final_yaw(0, 0)
            << std::endl;
  std::cout << "yaw:" << yaw.transpose() << std::endl;
  std::cout << "thetas_in: \n" << init_thetas_.transpose() << std::endl;
  std::cout << "thetas_out: \n" << final_thetas_.transpose() << std::endl;
  for (int i = 0; i < n_thetas_; ++i)
    std::cout << "thetas(" << i << "): " << thetas_vec[i].transpose()
              << std::endl;

  // P initial value
  // std::cout << "P initial value: " << std::endl;
  for (int i = 1; i < N_; ++i) {
    P.col(i - 1) = mid_q_vec[i - 1];
    std::cout << "P(" << i - 1 << "): " << P.col(i - 1).transpose()
              << std::endl;
  }
  Eigen::VectorXd T(N_);
  get_init_s4_taj(N_, P, T);
  backwardT(T, t);

  //! 3. opt begin
  std::cout << "iniState:\n" << iniState << std::endl;
  std::cout << "finState:\n" << finState << std::endl;
  minco_s4_opt_.reset(iniState.block<3, 4>(0, 0), finState.block<3, 4>(0, 0),
                      N_);
  minco_s4_opt_.generate(P, T);
  auto traj_info = minco_s4_opt_.getTraj();

  minco_s2_yaw_opt_.reset(init_yaw_, final_yaw_, N_);
  INFO_MSG_GREEN_BG("[GenTraj] 3 Value init done");

  // ! get sample pts
  std::vector<Eigen::Vector3d> sample_pts;
  // method 1 获取前1s轨迹 采样点 aabb
  // double t_end = traj_info.getTotalDuration() > 3.0 ? 3.0
  // :traj_info.getTotalDuration(); double dt = 0.2; for (double t = 0; t <
  // t_end; t += dt) //遍历轨迹
  //   sample_pts.push_back(traj_info.getPos(t));
  // sample_pts.push_back(traj_info.getPos(t_end));
  // gridmapPtr_->getAABBPointsSample(aabb_pts_vec_, sample_pts,
  // Eigen::Vector3d(1.5, 1.5, 4.5)); std::cout << "getAABBPointsSample get " <<
  // aabb_pts_vec_.size() << " pts" << std::endl;

  // method 2 获取轨迹完整aabb
  std::cout << "getAABBPoints full traj" << std::endl;
  gridmapPtr_->getAABBPoints(aabb_pts_group1_vec_, aabb_pts_group2_vec_,
                             iniState.col(0), finState.col(0),
                             Eigen::Vector3d(2.5, 2.5, 4.5));
  aabb_pts_vec_.clear();
  aabb_pts_vec_.reserve(aabb_pts_group1_vec_.size() +
                        aabb_pts_group2_vec_.size());
  aabb_pts_vec_.insert(aabb_pts_vec_.end(), aabb_pts_group1_vec_.begin(),
                       aabb_pts_group1_vec_.end());
  aabb_pts_vec_.insert(aabb_pts_vec_.end(), aabb_pts_group2_vec_.begin(),
                       aabb_pts_group2_vec_.end());
  visPtr_->visualize_pointcloud(aabb_pts_vec_, "AABB_pts");

  // ! start opt
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-16;
  lbfgs_params.delta = 1e-4;
  lbfgs_params.line_search_type = 0;
  lbfgs_params.max_linesearch = 120;
  lbfgs_params.max_iterations = max_iter_;
  irl_minco_debug_reset_markers();
  auto run_opt = [&](const bool is_warm_opt, const std::string& tag) {
    set_warm_opt(is_warm_opt);
    irl_minco_debug_stage_ = tag;
    auto tic = std::chrono::steady_clock::now();
    tictoc_innerloop_ = 0;
    tictoc_integral_ = 0;
    INFO_MSG_GREEN("[GenTraj] Begin opt (" << tag << ")");
    iter_times_ = 0;
    double minObjStage;
    int ret = lbfgs::lbfgs_optimize(dim_t_ + (4 + n_thetas_) * dim_p_, x_,
                                    &minObjStage, &objectiveFuncGoal, nullptr,
                                    &earlyExitGoal, this, &lbfgs_params);
    auto toc = std::chrono::steady_clock::now();
    INFO_MSG_GREEN_BG("[GenTraj] Opt (" << tag
                                        << ") done, ret=" << ret
                                        << ", objective=" << minObjStage
                                        << ", elapsed="
                                        << (toc - tic).count() * 1e-6
                                        << "ms"
                                        << ", collision_cost="
                                        << cost_collision_rec_);
    return ret;
  };

  auto is_nonfatal_opt_stop = [](const int ret) {
    return ret >= 0 || ret == lbfgs::LBFGSERR_MAXIMUMLINESEARCH;
  };

  bool reuse_cached_warm_start = false;
  if (warm_start_cache_matches(iniState, finState, init_yaw_, final_yaw_,
                               init_thetas_, final_thetas_, ego_path)) {
    std::copy(cached_warm_start_x_.begin(), cached_warm_start_x_.end(), x_);
    reuse_cached_warm_start = true;
    ROS_INFO("[TrajOpt] reuse cached stage-1 warm-start%s; skip stage1_no_sdf.",
             minco_warm_start_cache_force_reuse_ ? " (force)" : "");
  }

  int opt_ret_stage1 = 0;
  if (!reuse_cached_warm_start) {
    // stage-1: no collision gradient (warm start)
    opt_ret_stage1 = run_opt(true, "stage1_no_sdf");
    if (!is_nonfatal_opt_stop(opt_ret_stage1)) {
      delete[] x_;
      return false;
    }
    store_warm_start_cache(iniState, finState, init_yaw_, final_yaw_,
                           init_thetas_, final_thetas_, ego_path, opt_dim);
  }

  // stage-2: enable collision gradient, start from stage-1 solution
  int opt_ret_stage2 = run_opt(false, "stage2_with_sdf");
  irl_minco_debug_stage_ = "final";
  // cost_lock_.clear();

  INFO_MSG_GREEN_BG("[GenTraj] 5 Opt done");

  std::cout << "innerloop costs: " << tictoc_innerloop_ * 1e-6 << "ms"
            << std::endl;
  std::cout << "integral costs: " << tictoc_integral_ * 1e-6 << "ms"
            << std::endl;
  std::cout << "\033[32m>ret(stage2): " << opt_ret_stage2 << "\033[0m"
            << std::endl;

  dashboard_cost_print();
  INFO_MSG("iter: " << iter_times_);

  forwardT(t, T);
  minco_s4_opt_.generate(P, T);
  minco_s2_yaw_opt_.generate(yaw, T);

  for (int i = 0; i < n_thetas_; ++i) {
    minco_s2_theta_opt_vec_[i].generate(thetas_vec[i], T);
  }

  traj = getS4TrajWithYawAndThetas(minco_s4_opt_, minco_s2_yaw_opt_,
                                   minco_s2_theta_opt_vec_);

  bool opt_success = is_nonfatal_opt_stop(opt_ret_stage2);
  if (opt_success) {
    double min_irl_margin = 0.0;
    Eigen::Vector3d min_irl_margin_pos = Eigen::Vector3d::Zero();
    int min_irl_margin_group = 0;
    opt_success =
        is_irl_sphere_traj_valid(traj, 0.01, &min_irl_margin,
                                 &min_irl_margin_pos, &min_irl_margin_group);
    if (!opt_success) {
      INFO_MSG_RED("[GenTraj] reject trajectory by IRL sphere hard check, "
                   "group="
                   << min_irl_margin_group << ", margin=" << min_irl_margin
                   << ", pos=" << min_irl_margin_pos.transpose());
    }
  }

  irl_minco_debug_snapshot_traj(
      traj, "final", iter_times_,
      cost_snap_rec_ + cost_collision_rec_ + cost_wp_rec_ + cost_yaw_rec_ +
          cost_dyaw_rec_,
      0.0, 0, true, opt_success ? "success" : "fail_or_reject");

  delete[] x_;
  
  return opt_success;
}

inline int earlyExitGoal(void *ptrObj, const double *x, const double *grad,
                         const double fx, const double xnorm,
                         const double gnorm, const double step, int n, int k,
                         int ls) {
  TrajOpt &obj = *(TrajOpt *)ptrObj;
  obj.irl_minco_debug_snapshot_from_x(x, fx, step, k, ls, false, "running");

  if (obj.pause_debug_) {
    INFO_MSG_RED("earlyExit iter: " << obj.iter_times_);
    Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    Eigen::Map<const Eigen::MatrixXd> yaw(x + obj.dim_t_ + obj.dim_p_ * 3, 1,
                                          obj.dim_p_);
    std::vector<Eigen::Map<Eigen::VectorXd>> thetas_vec;
    for (int i = 0; i < obj.n_thetas_; ++i)
      thetas_vec.push_back(Eigen::Map<Eigen::VectorXd>(
          obj.x_ + obj.dim_t_ + (4 + i) * obj.dim_p_, obj.dim_p_));

    Eigen::VectorXd T(obj.N_);
    forwardT(t, T);

    obj.minco_s4_opt_.generate(P, T);
    obj.minco_s2_yaw_opt_.generate(yaw, T);
    for (int i = 0; i < obj.n_thetas_; ++i) {
      obj.minco_s2_theta_opt_vec_[i].generate(thetas_vec[i], T);
    }

    auto traj = getS4TrajWithYawAndThetas(
        obj.minco_s4_opt_, obj.minco_s2_yaw_opt_, obj.minco_s2_theta_opt_vec_);
    INFO_MSG_GREEN("traj type: " << traj.getType());
    obj.visPtr_->visualize_traj(traj, "debug_traj");
    std::vector<Eigen::Vector3d> int_waypts;
    for (const auto &piece : traj) {
      const auto &dur = piece.getDuration();
      for (int i = 0; i < obj.K_; ++i) {
        double t = dur * i / obj.K_;
        int_waypts.push_back(piece.getPos(t));
      }
    }
    obj.visPtr_->visualize_pointcloud(int_waypts, "int_waypts");

    //! check grad
    Eigen::VectorXd grad_arm_angles;
    Eigen::Vector4d grad_quat, grad_collision_quat;
    double grad_pos_yaw;
    double cost_collision_p, costp;
    Eigen::Vector3d grad_collision_p, gradp;
    Eigen::Vector3d p_end;

    Eigen::Vector3d grad_collision_p_K, gradp_K;
    double cost_collision_p_K, costp_K;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> gd_p_arr, gd_q_arr,
        gd_p_goal_arr, gd_thetas_arr, gd_quat_arr;
    visualization_msgs::MarkerArray robot_markers, rob_marker_wp;
    std::vector<Eigen::Vector3d> pcl_end, pcl_pen;
    std::vector<Eigen::Vector3d> end_path;

    size_t i_pie = 0;
    for (const auto &piece : traj) {
      gradp.setZero();
      costp = 0;
      grad_quat.setZero();

      const auto &dur = piece.getDuration();

      for (int i = 0; i < obj.K_; ++i) {
        double t = dur * i / obj.K_;
        Eigen::Vector3d pos = piece.getPos(t);
        Eigen::Vector3d vel = piece.getVel(t);
        Eigen::Vector3d acc = piece.getAcc(t);
        Eigen::Vector3d jer = piece.getJer(t);
        double yaw = piece.getAngle(t);
        double dyaw = piece.getAngleRate(t);
        Eigen::VectorXd arm_angles = piece.getTheta(t);

        Eigen::Vector4d quat;
        double thr;
        Eigen::Vector3d bodyrate;
        // obj.flatmap_.optimizated_forward(vel, acc, jer, quat);
        obj.flatmap_.forward(vel, acc, jer, yaw, dyaw, thr, quat, bodyrate);
        Eigen::Quaterniond q_b2w(quat[0], quat[1], quat[2], quat[3]);

        gradp_K.setZero();
        costp_K = 0;

        if (!obj.is_warm_opt_) {
          for (int idx = 0; idx < obj.aabb_pts_vec_.size(); ++idx) {
            double dist_threshold_backup = obj.dist_threshold_;
            if ((obj.aabb_pts_vec_[idx] - obj.end_pos_goal_).norm() <=
                obj.dist_threshold_end_range_) {
              obj.dist_threshold_ = obj.dist_threshold_end_;
            }
            if (obj.grad_sdf_full_state(obj.aabb_pts_vec_[idx], pos, quat,
                                        arm_angles, grad_collision_p,
                                        grad_collision_quat, grad_arm_angles,
                                        cost_collision_p)) {
              gradp += grad_collision_p;
              grad_quat += grad_collision_quat;
              costp += cost_collision_p;

              gradp_K += grad_collision_p;
              costp_K += cost_collision_p;
              pcl_pen.push_back(obj.aabb_pts_vec_[idx]);
            }
            obj.dist_threshold_ = dist_threshold_backup;
          }
        }
        Eigen::Vector3d p_e;
        Eigen::Quaterniond q_e2w;
        Eigen::Matrix4d T_e2b;
        obj.rc_sdf_ptr_->kine_ptr_->getEndPose(pos, q_b2w, arm_angles, p_e,
                                               q_e2w, T_e2b);
        end_path.push_back(p_e);

        Eigen::VectorXd thetas_temp(arm_angles); // thetas_temp.setZero();
        if (obj.en_tail_constraint_ && i_pie == traj.getPieceNum() - 2 &&
            i == 0) {
          if (obj.grad_end_pose(obj.p_end_tail_, obj.q_end_end_,
                                obj.p_body_tail_, pos, quat, thetas_temp, yaw,
                                p_end, grad_collision_p, grad_collision_quat,
                                grad_arm_angles, cost_collision_p)) {
            gradp += grad_collision_p;
            costp += cost_collision_p;
            std::cout << "p_end_tail:" << obj.p_end_tail_.transpose()
                      << ", p_end:" << p_end.transpose() << std::endl;
            std::cout << "cost_p:" << cost_collision_p
                      << ", grad_p:" << grad_collision_p.transpose()
                      << std::endl;

            // * get grad_pos marker
            Eigen::Vector3d arr_out, arr_in;
            arr_in = p_end;
            arr_out = p_end + grad_collision_p;
            gd_p_goal_arr.push_back(std::make_pair(arr_in, arr_out));
            pcl_end.push_back(p_end);
            std::cout << "[gd_p] arr_in:" << arr_in.transpose()
                      << ", arr_out:" << arr_out.transpose() << std::endl;
            obj.rc_sdf_ptr_->getRobotMarkerArray(
                pos, q_b2w, arm_angles, rob_marker_wp,
                visualization_rc_sdf::Color::blue, 0.1);

            // * get grad_thetas marker
            Eigen::Matrix4d T_b2w, T_cur2w, T_cur2b;
            T_b2w.setIdentity();
            T_b2w.block<3, 3>(0, 0) = q_b2w.toRotationMatrix();
            T_b2w.block<3, 1>(0, 3) = pos;
            for (size_t k = 0; k < obj.rc_sdf_ptr_->getBoxNum() - 1; ++k) {
              obj.rc_sdf_ptr_->kine_ptr_->getRelativeTransform(
                  arm_angles, k + 1, 0, T_cur2b);
              T_cur2w = T_b2w * T_cur2b;
              arr_in = T_cur2w.block<3, 1>(0, 3);

              size_t axis;
              axis = (k == 0 || k == 1) ? 1 : 0;
              arr_out = arr_in + grad_arm_angles(k) *
                                     T_cur2w.block<3, 1>(0, axis) * 1e-5;
              gd_thetas_arr.push_back(std::make_pair(arr_in, arr_out));
            }

            // * get grad_quat marker
            arr_in = pos;
            arr_out =
                pos + obj.quatGradToAxisGradient(q_b2w, grad_collision_quat);
            gd_quat_arr.push_back(std::make_pair(arr_in, arr_out));

            obj.rc_sdf_ptr_->robotMarkersPub(rob_marker_wp,
                                             "debug_rob_marker_wp");
            // obj.visPtr_->visualize_pointcloud(, "debug_p_end_tail");
            obj.visPtr_->visualize_pointcloud(pcl_end, "debug_pcl_end");
            obj.visPtr_->visualize_arrows(gd_quat_arr, "debug_gd_quat");
            obj.visPtr_->visualize_arrows(gd_p_goal_arr, "debug_grad_goal_p");
            obj.visPtr_->visualize_arrows(gd_thetas_arr, "debug_grad_thetas");
          }
        }

        if (costp_K > DBL_EPSILON) {
          gd_p_arr.push_back(
              std::make_pair(pos, pos + costp_K * gradp_K * 1e-5));
          // std::cout << "in:" << pos.transpose() << ", out:" <<
          // (pos+costp_K*gradp_K*1e-5).transpose() << std::endl;
          // gd_q_arr.push_back(std::make_pair(quat,grad_quat));
        }
        obj.rc_sdf_ptr_->getRobotMarkerArray(
            pos, q_b2w, arm_angles, robot_markers,
            visualization_rc_sdf::Color::blue, 0.1);
      }
      i_pie += 1;
    }
    std::cout << "gd_p_arr.size:" << gd_p_arr.size() << std::endl;
    obj.visPtr_->visualize_pointcloud(pcl_pen, "debug_pcl_pen");
    obj.visPtr_->visualize_arrows(gd_p_arr, "debug_grad_collision_p");
    obj.visPtr_->visualize_arrows(gd_q_arr, "debug_grad_collision_q");
    obj.rc_sdf_ptr_->robotMarkersPub(robot_markers, "debug_robot_markers");

    // std::vector<Eigen::Vector3d> aabb_pts_vec;
    // for(int i=0; i<obj.aabb_pts_.cols();++i)
    //   aabb_pts_vec.push_back(obj.aabb_pts_.col(i));
    // obj.visPtr_->visualize_pointcloud(aabb_pts_vec, "aabb_pts");

    INFO_MSG_YELLOW("in process");
    obj.dashboard_cost_print();
    // int a;
    // std::cin >> a;

    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return 0;
}


void TrajOpt::sample_traj_states(
    const Trajectory<7> &traj, std::vector<Eigen::Vector3d> &pos_seq,
    std::vector<Eigen::Quaterniond> &q_b2w_seq,
    std::vector<Eigen::VectorXd> &arm_angles_seq) {
  pos_seq.clear();
  q_b2w_seq.clear();
  arm_angles_seq.clear();

  if (traj.getType() != TrajType::WITHYAWANDTHETA) {
    ROS_WARN_STREAM("[TrajOpt] sample_traj_states expects WITHYAWANDTHETA "
                    "trajectory, got type " << traj.getType());
  }

  // per-piece uniform sampling (same stride as debug in earlyExitGoal)
  for (const auto &piece : traj) {
    const double dur = piece.getDuration();
    for (int i = 0; i < K_; ++i) {
      const double t = dur * static_cast<double>(i) / static_cast<double>(K_);

      const Eigen::Vector3d pos = piece.getPos(t);
      const Eigen::Vector3d vel = piece.getVel(t);
      const Eigen::Vector3d acc = piece.getAcc(t);
      const Eigen::Vector3d jer = piece.getJer(t);
      const double yaw = piece.getAngle(t);
      const double dyaw = piece.getAngleRate(t);
      const Eigen::VectorXd arm_angles = piece.getTheta(t);

      Eigen::Vector4d quat_raw;
      Eigen::Vector3d bodyrate;
      double thr; // unused but required by forward
      flatmap_.forward(vel, acc, jer, yaw, dyaw, thr, quat_raw, bodyrate);
      Eigen::Quaterniond q_b2w(quat_raw[0], quat_raw[1], quat_raw[2],
                               quat_raw[3]);

      pos_seq.push_back(pos);
      q_b2w_seq.push_back(q_b2w);
      arm_angles_seq.push_back(arm_angles);
    }
  }

  // append exact end point to ensure coverage
  if (traj.getPieceNum() > 0) {
    const auto &last_piece = traj[traj.getPieceNum() - 1];
    const double t_end = last_piece.getDuration();

    const Eigen::Vector3d pos = last_piece.getPos(t_end);
    const Eigen::Vector3d vel = last_piece.getVel(t_end);
    const Eigen::Vector3d acc = last_piece.getAcc(t_end);
    const Eigen::Vector3d jer = last_piece.getJer(t_end);
    const double yaw = last_piece.getAngle(t_end);
    const double dyaw = last_piece.getAngleRate(t_end);
    const Eigen::VectorXd arm_angles = last_piece.getTheta(t_end);

    Eigen::Vector4d quat_raw;
    Eigen::Vector3d bodyrate;
    double thr;
    flatmap_.forward(vel, acc, jer, yaw, dyaw, thr, quat_raw, bodyrate);
    Eigen::Quaterniond q_b2w(quat_raw[0], quat_raw[1], quat_raw[2],
                             quat_raw[3]);

    pos_seq.push_back(pos);
    q_b2w_seq.push_back(q_b2w);
    arm_angles_seq.push_back(arm_angles);
  }
}

bool TrajOpt::irl_minco_debug_build_traj_from_x(const double *x,
                                                Trajectory<7> &traj) {
  if (x == nullptr || N_ <= 0 || dim_t_ <= 0 || dim_p_ < 0) {
    return false;
  }

  Eigen::Map<const Eigen::VectorXd> t(x, dim_t_);
  Eigen::Map<const Eigen::MatrixXd> P(x + dim_t_, 3, dim_p_);
  Eigen::Map<const Eigen::MatrixXd> yaw(x + dim_t_ + dim_p_ * 3, 1, dim_p_);

  Eigen::VectorXd T(N_);
  forwardT(t, T);

  minco_s4_opt_.generate(P, T);
  minco_s2_yaw_opt_.generate(yaw, T);
  for (int i = 0; i < n_thetas_; ++i) {
    Eigen::Map<const Eigen::MatrixXd> theta(
        x + dim_t_ + (4 + i) * dim_p_, 1, dim_p_);
    minco_s2_theta_opt_vec_[i].generate(theta, T);
  }

  traj = getS4TrajWithYawAndThetas(minco_s4_opt_, minco_s2_yaw_opt_,
                                   minco_s2_theta_opt_vec_);
  return true;
}

void TrajOpt::irl_minco_debug_reset_markers() {
  if (!irl_minco_debug_enable_ || !pub_irl_minco_debug_markers_) {
    return;
  }

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(marker);
  pub_irl_minco_debug_markers_.publish(markers);
  irl_minco_debug_marker_seq_ = 0;
}

void TrajOpt::irl_minco_debug_snapshot_from_x(const double *x, double fx,
                                              double step, int lbfgs_iter,
                                              int ls, bool force,
                                              const std::string &status) {
  if (!irl_minco_debug_enable_) {
    return;
  }

  const int iter = lbfgs_iter > 0 ? lbfgs_iter : iter_times_;
  const bool wants_print =
      force || iter <= 1 || iter % irl_minco_debug_print_stride_ == 0;
  const bool wants_marker =
      force || iter <= 1 || iter % irl_minco_debug_marker_stride_ == 0;
  if (!wants_print && !wants_marker) {
    return;
  }

  Trajectory<7> traj;
  if (!irl_minco_debug_build_traj_from_x(x, traj)) {
    return;
  }
  irl_minco_debug_snapshot_traj(traj, irl_minco_debug_stage_, lbfgs_iter, fx,
                                step, ls, force, status);
}

void TrajOpt::irl_minco_debug_snapshot_traj(const Trajectory<7> &traj,
                                            const std::string &stage,
                                            int lbfgs_iter, double fx,
                                            double step, int ls, bool force,
                                            const std::string &status) {
  if (!irl_minco_debug_enable_) {
    return;
  }

  const int iter = lbfgs_iter > 0 ? lbfgs_iter : iter_times_;
  std::vector<Eigen::Vector3d> points;
  sampleDebugTrajectory(traj, irl_minco_debug_sample_dt_, points);
  const IrlMincoDebugMetrics metrics =
      computeIrlMincoDebugMetrics(*this, points);
  const bool warn = metrics.negative_margin || metrics.possible_loop ||
                    status.find("fail") != std::string::npos ||
                    status.find("reject") != std::string::npos;

  const bool should_print =
      force || warn || iter <= 1 || iter % irl_minco_debug_print_stride_ == 0;
  if (should_print) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4)
       << "[MINCO-IRL] stage=" << stage << " iter=" << iter
       << " eval=" << iter_times_ << " fx=" << fx << " step=" << step
       << " ls=" << ls << " d=(" << irl_safety_distance_1_ << ","
       << irl_safety_distance_2_ << ") cost{snap=" << cost_snap_rec_
       << ", coll=" << cost_collision_rec_ << ", guide=" << cost_wp_rec_
       << ", yaw=" << cost_yaw_rec_ << ", dyaw=" << cost_dyaw_rec_ << "}"
       << " clearance=(" << finiteOrDash(metrics.min_clearance_1) << ","
       << finiteOrDash(metrics.min_clearance_2) << ")"
       << " margin=(" << finiteOrDash(metrics.min_margin_1) << ","
       << finiteOrDash(metrics.min_margin_2) << ")"
       << " len=" << metrics.path_len << " chord=" << metrics.chord_len
       << " ratio=" << metrics.length_ratio
       << " reverse=" << metrics.reverse_count
       << " self_cross=" << metrics.self_cross_count
       << " worst_group=" << metrics.worst_group
       << " status=" << (status.empty() ? "running" : status);
    if (warn) {
      ROS_WARN_STREAM(ss.str());
    } else {
      ROS_INFO_STREAM(ss.str());
    }
  }

  const bool should_publish_marker =
      force || warn || iter <= 1 || iter % irl_minco_debug_marker_stride_ == 0;
  if (!should_publish_marker || !pub_irl_minco_debug_markers_ ||
      points.size() < 2) {
    return;
  }
  if (!force && !warn &&
      irl_minco_debug_marker_seq_ >= irl_minco_debug_max_markers_) {
    return;
  }

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker line;
  line.header.stamp = ros::Time::now();
  line.header.frame_id = "world";
  line.ns = "irl_minco_inner_iterations";
  line.id = irl_minco_debug_marker_seq_ * 10;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = force ? 0.065 : (warn ? 0.05 : 0.032);
  setDebugColor(line, stage, force, warn);
  line.points.reserve(points.size());
  for (const auto &p : points) {
    geometry_msgs::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z() + (force ? 0.10 : 0.06);
    line.points.push_back(point);
  }
  markers.markers.push_back(line);

  if (warn && metrics.worst_group > 0) {
    visualization_msgs::Marker sphere;
    sphere.header = line.header;
    sphere.ns = "irl_minco_inner_worst_points";
    sphere.id = irl_minco_debug_marker_seq_ * 10 + 1;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.pose.position.x = metrics.worst_pos.x();
    sphere.pose.position.y = metrics.worst_pos.y();
    sphere.pose.position.z = metrics.worst_pos.z() + 0.12;
    sphere.scale.x = 0.16;
    sphere.scale.y = 0.16;
    sphere.scale.z = 0.16;
    sphere.color.r = 1.0;
    sphere.color.g = 0.02;
    sphere.color.b = 0.02;
    sphere.color.a = 0.9;
    markers.markers.push_back(sphere);
  }

  pub_irl_minco_debug_markers_.publish(markers);
  irl_minco_debug_marker_seq_ += 1;
}


// SECTION object function
inline double objectiveFuncGoal(void *ptrObj, const double *x, double *grad,
                                const int n) {
  TrajOpt &obj = *(TrajOpt *)ptrObj;
  obj.iter_times_++;
  obj.clear_cost_rec();
  INFO_MSG_RED("iter: " << obj.iter_times_);

  int n_thetas = obj.minco_s2_theta_opt_vec_.size();

  //! 1. fetch opt varaibles from x_
  Eigen::Map<const Eigen::VectorXd> t(x, obj.dim_t_);
  Eigen::Map<Eigen::VectorXd> gradt(grad, obj.dim_t_);

  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_);

  Eigen::Map<const Eigen::MatrixXd> yaw(x + obj.dim_t_ + obj.dim_p_ * 3, 1,
                                        obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> grad_yaw(grad + obj.dim_t_ + obj.dim_p_ * 3, 1,
                                       obj.dim_p_);

  std::vector<Eigen::Map<const Eigen::MatrixXd>> theta_vec;
  std::vector<Eigen::Map<Eigen::MatrixXd>> grad_theta_vec;
  for (int i = 0; i < n_thetas; ++i) {
    theta_vec.push_back(Eigen::Map<const Eigen::MatrixXd>(
        x + obj.dim_t_ + (4 + i) * obj.dim_p_, 1, obj.dim_p_));
    grad_theta_vec.push_back(Eigen::Map<Eigen::MatrixXd>(
        grad + obj.dim_t_ + (4 + i) * obj.dim_p_, 1, obj.dim_p_));
  }

  //! 2. reform P & T
  Eigen::VectorXd T(obj.N_);
  forwardT(t, T);

  //! 3. generate minco using P & T
  auto tic = std::chrono::steady_clock::now();
  obj.minco_s4_opt_.generate(P, T);
  obj.minco_s2_yaw_opt_.generate(yaw, T);
  for (int i = 0; i < n_thetas; ++i)
    obj.minco_s2_theta_opt_vec_[i].generate(theta_vec[i], T);

  double cost = obj.minco_s4_opt_.getTrajSnapCost();
  obj.minco_s4_opt_.calGrads_CT();
  obj.cost_snap_rec_ = cost;

  cost += obj.minco_s2_yaw_opt_.getEnergyCost();
  obj.minco_s2_yaw_opt_.calGrads_CT();

  for (int i = 0; i < n_thetas; ++i) {
    cost += obj.minco_s2_theta_opt_vec_[i].getEnergyCost();
    obj.minco_s2_theta_opt_vec_[i].calGrads_CT();
  }
  auto toc = std::chrono::steady_clock::now();
  INFO_MSG_GREEN("time generate: " << (toc - tic).count() * 1e-6 << "ms");

  //! 4. calculate penalty and gradient to C & T
  tic = std::chrono::steady_clock::now();
  obj.addTimeIntPenaltyGoal(cost);
  toc = std::chrono::steady_clock::now();
  tictoc_integral_ += (toc - tic).count();
  INFO_MSG_GREEN("time int penalty: " << (toc - tic).count() * 1e-6 << "ms");

  //! 5. propogate gradient to mid-point P & T
  tic = std::chrono::steady_clock::now();
  obj.minco_s4_opt_.calGrads_PT();
  obj.minco_s2_yaw_opt_.calGrads_PT();
  for (int i = 0; i < n_thetas; ++i) {
    obj.minco_s2_theta_opt_vec_[i].calGrads_PT();
  }
  toc = std::chrono::steady_clock::now();
  tictoc_innerloop_ += (toc - tic).count();

  //! 6. propogate gradient to opt variable P & tau
  obj.minco_s4_opt_.gdT.array() += obj.rhoT_;
  cost += obj.rhoT_ * T.sum();
  obj.cost_t_rec_ = obj.rhoT_ * T.sum();

  auto gdT = obj.minco_s4_opt_.gdT;
  gdT += obj.minco_s2_yaw_opt_.gdT;
  for (int i = 0; i < n_thetas; ++i)
    gdT += obj.minco_s2_theta_opt_vec_[i].gdT;

  addLayerTGrad(t, gdT, gradt);
  gradP = obj.minco_s4_opt_.gdP;
  grad_yaw = obj.minco_s2_yaw_opt_.gdP;
  for (int i = 0; i < n_thetas; ++i)
    grad_theta_vec[i] = obj.minco_s2_theta_opt_vec_[i].gdP;

  return cost;
}

void TrajOpt::addTimeIntPenaltyGoal(double &cost) {
  // cost of one inner sample points
  double cost_inner;
  // state of innner sample point
  Eigen::Vector3d pos, vel, acc, jer, sna;
  double yaw, dyaw, ddyaw;
  // gradient of the state of innner sample point
  Eigen::Vector3d grad_p, grad_v, grad_a, grad_j, grad_omega;
  double grad_yaw, grad_dyaw;
  // derivatives of the time base
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  Eigen::Matrix<double, 4, 1> beta_yaw0, beta_yaw1, beta_yaw2;
  // sample time step; alpha; trapezoidal integral weights
  double step, alpha, omg;
  // current sample time within the segment
  double st = 0.0;
  // gradient propogate to C & T of each segment
  Eigen::Matrix<double, 8, 3> gradViola_c;
  double gradViola_t;
  Eigen::Matrix<double, 4, 1> gradViola_c_yaw;
  double gradViola_t_yaw;
  // exact term cost and gradient
  double cost_vel, cost_acc, cost_collision_p, cost_vel_yaw, cost_yaw;
  Eigen::Vector3d grad_vel, grad_acc, grad_collision_p;
  double grad_vel_yaw;
  double grad_pos_yaw;
  // total gradient
  Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
  double totalGradPsi = 0.0, totalGradPsiD = 0.0;
  // collision related
  double thr = 0.0;
  Eigen::Vector4d quat;
  Eigen::Vector3d bodyrate;
  Eigen::Vector4d grad_quat, grad_collision_quat;
  // exact term cost and gradient
  double cost_rate = 0.0;
  Eigen::Vector3d grad_rate;
  double grad_thr;
  // arm angles
  int n_thetas = minco_s2_theta_opt_vec_.size();
  Eigen::VectorXd thetas(n_thetas), dthetas(n_thetas), ddthetas(n_thetas);
  Eigen::VectorXd grad_thetas(n_thetas), grad_dthetas(n_thetas);
  std::vector<Eigen::Matrix<double, 4, 1>> gradViola_c_theta_vec(n_thetas);
  Eigen::VectorXd gradViola_t_thetas(n_thetas);
  Eigen::VectorXd grad_vel_thetas(n_thetas), grad_collision_thetas(n_thetas);
  Eigen::VectorXd cost_vel_thetas(n_thetas);
  Eigen::Vector3d p_end;

  int innerLoop = K_ + 1;
  bool enable_irl_sphere_collision = false;
  bool enable_irl_analytic_cylinder_collision = false;
  double irl_uam_sphere_radius = 0.0;
  double irl_safety_distance_1 = 0.0;
  double irl_safety_distance_2 = 0.0;
  Eigen::Vector2d irl_cylinder1_center_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d irl_cylinder2_center_xy = Eigen::Vector2d::Zero();
  double irl_cylinder1_radius = 0.0;
  double irl_cylinder2_radius = 0.0;
  double irl_cylinder_z_min = 0.0;
  double irl_cylinder_z_max = 0.0;
  {
    std::lock_guard<std::mutex> lock(irl_sphere_collision_mutex_);
    enable_irl_sphere_collision = enable_irl_sphere_collision_;
    enable_irl_analytic_cylinder_collision =
        enable_irl_analytic_cylinder_collision_;
    irl_uam_sphere_radius = irl_uam_sphere_radius_;
    irl_safety_distance_1 = irl_safety_distance_1_;
    irl_safety_distance_2 = irl_safety_distance_2_;
    irl_cylinder1_center_xy = irl_cylinder1_center_xy_;
    irl_cylinder2_center_xy = irl_cylinder2_center_xy_;
    irl_cylinder1_radius = irl_cylinder1_radius_;
    irl_cylinder2_radius = irl_cylinder2_radius_;
    irl_cylinder_z_min = irl_cylinder_z_min_;
    irl_cylinder_z_max = irl_cylinder_z_max_;
  }

  // ROS_WARN("!!!!!!!!!!!!!n= %d", N_);
  // std::cout << "minco_s4_opt_.T1:" << minco_s4_opt_.T1.transpose() <<
  // std::endl;
  for (int i = 0; i < N_; ++i) {
    step = minco_s4_opt_.T1(i) / K_;
    const auto &c = minco_s4_opt_.b.block<8, 3>(i * 8, 0);
    const auto &c_yaw = minco_s2_yaw_opt_.b.block<4, 1>(i * 4, 0);
    std::vector<Eigen::Matrix<double, 4, 1>> c_theta_vec;
    for (size_t j = 0; j < n_thetas; ++j)
      c_theta_vec.push_back(minco_s2_theta_opt_vec_[j].b.block<4, 1>(i * 4, 0));

    st = 0.0;

    for (int j = 0; j < innerLoop; ++j) {
      omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;
      alpha = 1.0 / K_ * j;
      beta0 = cal_timebase_snap(0, st);
      beta1 = cal_timebase_snap(1, st);
      beta2 = cal_timebase_snap(2, st);
      beta3 = cal_timebase_snap(3, st);
      beta4 = cal_timebase_snap(4, st);
      beta_yaw0 = cal_timebase_acc(0, st);
      beta_yaw1 = cal_timebase_acc(1, st);
      beta_yaw2 = cal_timebase_acc(2, st);
      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      sna = c.transpose() * beta4;
      yaw = c_yaw.transpose() * beta_yaw0;
      dyaw = c_yaw.transpose() * beta_yaw1;
      ddyaw = c_yaw.transpose() * beta_yaw2;

      // std::cout << "[" << i << "," << j << "," << st << "] p:" <<
      // pos.transpose() << std::endl;

      for (size_t k = 0; k < n_thetas; ++k) {
        thetas(k) = c_theta_vec[k].transpose() * beta_yaw0;
        dthetas(k) = c_theta_vec[k].transpose() * beta_yaw1;
        ddthetas(k) = c_theta_vec[k].transpose() * beta_yaw2;
      }

      //! isOccupied_se3 也调用了
      flatmap_.forward(vel, acc, jer, yaw, dyaw, thr, quat, bodyrate);

      gradViola_c.setZero();
      gradViola_t = 0.0;
      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      grad_j.setZero();
      grad_yaw = 0.0;
      grad_dyaw = 0.0;
      grad_quat.setZero();
      grad_omega.setZero();
      grad_thr = 0; // no use
      grad_thetas.setZero();
      grad_dthetas.setZero();
      grad_vel_thetas.setZero();
      cost_inner = 0.0;

      //! Benchmark
      //* Convex

      //* ESDF
      // if (grad_cost_collision(pos, grad_collision_p, cost_collision_p)) {
      //   grad_p += grad_collision_p;
      //   cost_inner += cost_collision_p;
      //   cost_collision_rec_ += omg * step * cost_collision_p;
      // }

      //* IRL sphere collision
      if (enable_irl_sphere_collision && !is_warm_opt_) {
        if (enable_irl_analytic_cylinder_collision) {
          if (grad_cost_analytic_cylinder_collision(
                  pos, irl_cylinder1_center_xy, irl_cylinder1_radius,
                  irl_cylinder_z_min, irl_cylinder_z_max,
                  irl_safety_distance_1, irl_uam_sphere_radius,
                  grad_collision_p, cost_collision_p)) {
            grad_p += grad_collision_p;
            cost_inner += cost_collision_p;
            cost_collision_rec_ += omg * step * cost_collision_p;
          }
          if (grad_cost_analytic_cylinder_collision(
                  pos, irl_cylinder2_center_xy, irl_cylinder2_radius,
                  irl_cylinder_z_min, irl_cylinder_z_max,
                  irl_safety_distance_2, irl_uam_sphere_radius,
                  grad_collision_p, cost_collision_p)) {
            grad_p += grad_collision_p;
            cost_inner += cost_collision_p;
            cost_collision_rec_ += omg * step * cost_collision_p;
          }
        } else {
          if (grad_cost_sphere_collision(pos, aabb_pts_group1_vec_,
                                         irl_safety_distance_1,
                                         irl_uam_sphere_radius,
                                         grad_collision_p, cost_collision_p)) {
            grad_p += grad_collision_p;
            cost_inner += cost_collision_p;
            cost_collision_rec_ += omg * step * cost_collision_p;
          }
          if (grad_cost_sphere_collision(pos, aabb_pts_group2_vec_,
                                         irl_safety_distance_2,
                                         irl_uam_sphere_radius,
                                         grad_collision_p, cost_collision_p)) {
            grad_p += grad_collision_p;
            cost_inner += cost_collision_p;
            cost_collision_rec_ += omg * step * cost_collision_p;
          }
        }
      }

      if (grad_cost_astar_waypoint_tracking(pos, grad_collision_p,
                                            cost_collision_p)) {
        grad_p += grad_collision_p;
        cost_inner += cost_collision_p;
        cost_wp_rec_ += omg * step * cost_collision_p;
      }

      Eigen::Vector3d grad_tmp, grad_tmp2;
      double cost_tmp;
      if (grad_cost_omega(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
        cost_omega_rec_ += omg * step * cost_tmp;
      }

      //! ring-through
      bool en_wp_constraint_ring = false;
      Eigen::Vector3d ref_pos;
      Eigen::VectorXd ref_thetas(n_thetas); ref_thetas.setZero();
      auto is_mid_point_hit = [&](int mid_idx) {
        return (i == mid_idx && j == innerLoop - 1) ||
               (i == mid_idx + 1 && j == 0);
      };
      if (en_through_ring_ && has_ring_pose_ && is_warm_opt_) {
        if (is_mid_point_hit(0)) {
          ref_pos = ring_mid_pts_[0];
          en_wp_constraint_ring = true;
        } else if (is_mid_point_hit(1)) {
          ref_pos = ring_mid_pts_[1];
          en_wp_constraint_ring = true;
        } else if (is_mid_point_hit(2)) {
          ref_pos = ring_mid_pts_[2];
          en_wp_constraint_ring = true;
        }
      }

      if (en_wp_constraint_ring) {
        Eigen::Vector4d ring_mid_q(ring_mid_quat_.w(), ring_mid_quat_.x(),
                                    ring_mid_quat_.y(), ring_mid_quat_.z());
        if (grad_cost_full_state(pos, quat, thetas, ref_pos, ring_mid_q,
                                 ref_thetas, grad_collision_p,
                                 grad_collision_quat, grad_collision_thetas,
                                 cost_collision_p)) {
          grad_p += grad_collision_p;
          grad_quat += grad_collision_quat;
          grad_thetas += grad_collision_thetas;

          cost_inner += cost_collision_p;
          cost_wp_rec_ += omg * step * cost_collision_p;
        }
      }

      //! Tail constraint
      bool en_tail_constraint_on_wp = false;
      Eigen::Vector3d p_end_wp;
      if (en_tail_constraint_) {
        if (i == N_ - 3 && j == innerLoop - 1) {
          p_end_wp = p_end_tail_;
          en_tail_constraint_on_wp = true;
        } else if (i == N_ - 2) {
          Eigen::Vector3d dir = (p_end_end_ - p_end_tail_).normalized();
          double dis = (double)j / (double)(innerLoop - 1) *
                       (p_end_end_ - p_end_tail_).norm();
          p_end_wp = p_end_tail_ + dir * dis;
          en_tail_constraint_on_wp = true;
        }
      }

      if (en_tail_constraint_on_wp){
        if(grad_end_pose(p_end_wp, q_end_end_, p_body_tail_, pos, quat, thetas,
                              yaw, p_end, grad_collision_p, grad_collision_quat,
                              grad_collision_thetas, cost_collision_p)) {
        grad_p += grad_collision_p;
        grad_quat += grad_collision_quat;
        grad_thetas += grad_collision_thetas;

        cost_inner += cost_collision_p;
        cost_wp_rec_ += omg * step * cost_collision_p;
        }

        Eigen::Vector3d grad_vel_tail;
        double cost_vel_tail;
        if (grad_cost_v(vel, v_max_tail_, rhoV_tail_, grad_vel_tail,
                        cost_vel_tail)) {
          grad_v += grad_vel_tail;
          cost_inner += cost_vel_tail;
          cost_v_rec_ += omg * step * cost_vel_tail;
        }

        Eigen::Vector3d grad_acc_tail;
        double cost_acc_tail;
        if (grad_cost_a(acc, a_max_tail_, rhoA_tail_, grad_acc_tail,
                        cost_acc_tail)) {
          grad_a += grad_acc_tail;
          cost_inner += cost_acc_tail;
          cost_a_rec_ += omg * step * cost_acc_tail;
        }
      }

      Eigen::VectorXd grad_cost_theta_vec(n_thetas);
      double cost_theta_total = 0;
      if (grad_cost_theta(thetas, grad_cost_theta_vec, cost_theta_total)) {
        grad_thetas += grad_cost_theta_vec;
        cost_inner += cost_theta_total;
        for (size_t k = 0; k < n_thetas; ++k) {
          cost_thetas_rec_(k) += omg * step * cost_theta_total / n_thetas;
        }
      }

      Eigen::VectorXd grad_cost_dtheta_vec(n_thetas);
      double cost_dtheta_total = 0;
      if (grad_cost_dtheta(dthetas, grad_cost_dtheta_vec, cost_dtheta_total)) {
        grad_dthetas += grad_cost_dtheta_vec;
        cost_inner += cost_dtheta_total;
        for (size_t k = 0; k < n_thetas; ++k) {
          cost_dthetas_rec_(k) += omg * step * cost_dtheta_total / n_thetas;
        }
      }

      if (grad_cost_v(vel, grad_vel, cost_vel)) {
        grad_v += grad_vel;
        cost_inner += cost_vel;
        cost_v_rec_ += omg * step * cost_vel;
      }
      if (grad_cost_a(acc, grad_acc, cost_acc)) {
        grad_a += grad_acc;
        cost_inner += cost_acc;
        cost_a_rec_ += omg * step * cost_acc;
      }
      if (grad_cost_dyaw(dyaw, grad_vel_yaw, cost_vel_yaw)) {
        grad_dyaw += grad_vel_yaw;
        cost_inner += cost_vel_yaw;
        cost_dyaw_rec_ += omg * step * cost_vel_yaw;
      }
      if (i > 1 && i < N_ - 1) { // ignore head and tail piece
        if (grad_cost_yaw_forward(yaw, vel, grad_pos_yaw, grad_vel, cost_yaw)) {
          grad_yaw += grad_pos_yaw;
          grad_v += grad_vel;
          cost_inner += cost_yaw;
          cost_yaw_rec_ += omg * step * cost_yaw;
        }
      }

      // gradViola_c = beta0 * grad_p.transpose();
      // gradViola_t = grad_p.transpose() * vel;
      // gradViola_c += beta1 * grad_v.transpose();
      // gradViola_t += grad_v.transpose() * acc;
      // gradViola_c += beta2 * grad_a.transpose();
      // gradViola_t += grad_a.transpose() * jer;
      // gradViola_c += beta3 * grad_j.transpose();
      // gradViola_t += grad_j.transpose() * sna;

      // gradViola_c_yaw = beta_yaw0 * grad_yaw;
      // gradViola_t_yaw = grad_yaw * dyaw;
      // gradViola_c_yaw += beta_yaw1 * grad_dyaw;
      // gradViola_t_yaw += grad_dyaw * ddyaw;

      flatmap_.backward(grad_p, grad_v, grad_a, grad_thr, grad_quat, grad_omega,
                        totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                        totalGradPsi, totalGradPsiD);

      totalGradPsi += grad_yaw;
      totalGradPsiD += grad_dyaw;

      gradViola_c = beta0 * totalGradPos.transpose();
      gradViola_t = totalGradPos.transpose() * vel;
      gradViola_c += beta1 * totalGradVel.transpose();
      gradViola_t += totalGradVel.transpose() * acc;
      gradViola_c += beta2 * totalGradAcc.transpose();
      gradViola_t += totalGradAcc.transpose() * jer;
      gradViola_c += beta3 * totalGradJer.transpose();
      gradViola_t += totalGradJer.transpose() * sna;

      gradViola_c_yaw = beta_yaw0 * totalGradPsi;
      gradViola_t_yaw = totalGradPsi * dyaw;
      gradViola_c_yaw += beta_yaw1 * totalGradPsiD;
      gradViola_t_yaw += totalGradPsiD * ddyaw;

      for (size_t k = 0; k < n_thetas; ++k) {
        gradViola_c_theta_vec[k] = beta_yaw0 * grad_thetas(k);
        // gradViola_t_thetas(k) = grad_thetas(k) * dthetas(k);
        gradViola_c_theta_vec[k] += beta_yaw1 * grad_dthetas(k);
        // gradViola_t_thetas(k) += grad_dthetas(k) * ddthetas(k);

        gradViola_t_yaw += grad_thetas(k) * dthetas(k);
        gradViola_t_yaw += grad_dthetas(k) * ddthetas(k);
      }

      minco_s4_opt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c;
      minco_s4_opt_.gdT(i) +=
          omg * (cost_inner / K_ + alpha * step * gradViola_t +
                 alpha * step * gradViola_t_yaw);
      minco_s2_yaw_opt_.gdC.block<4, 1>(i * 4, 0) +=
          omg * step * gradViola_c_yaw;
      for (size_t k = 0; k < n_thetas; ++k) {
        minco_s2_theta_opt_vec_[k].gdC.block<4, 1>(i * 4, 0) +=
            omg * step * gradViola_c_theta_vec[k];
      }

      cost += omg * step * cost_inner;
      st += step;
    }
  }
}
} // namespace traj_opt
