#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "map_interface/map_interface.h"
#include "minco.hpp"
#include "parameter_server/parameter_server.hpp"
#include "quadrotor_msgs/DebugOpt.h"
#include "rotation_util/rotation_util.hpp"
#include <geometry_msgs/PoseStamped.h>
#include "util_gym/data_manager.hpp"
#include "util_gym/util_gym.hpp"
#include "visualization_interface/vis_interface.h"
#include <ch_rc_sdf/ch_rc_sdf.h>
#include <visualization_msgs/MarkerArray.h>
#include <traj_opt/flatness.hpp>

#define USE_RC_SDF
// #define USE_ESDF

namespace traj_opt {

using rot_util = rotation_util::RotUtil;

class TrajOpt {
public:
  std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
  std::shared_ptr<vis_interface::VisInterface> visPtr_;
  std::shared_ptr<clutter_hand::CH_RC_SDF> rc_sdf_ptr_;

  std::vector<Eigen::Vector3d> inter_wps_;

  bool pause_debug_ = false;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // cost record
  std::atomic_flag cost_lock_ = ATOMIC_FLAG_INIT;
  double cost_t_rec_, cost_v_rec_, cost_a_rec_, cost_thrust_rec_,
      cost_omega_rec_, cost_perching_collision_rec_, cost_snap_rec_;
  double cost_perching_precep_rec_;
  double cost_tracking_dis_rec_, cost_tracking_ang_rec_, cost_tracking_vis_rec_,
      cost_collision_rec_;
  double cost_yaw_rec_, cost_dyaw_rec_;
  double deltaT_rec_;
  double cost_wp_rec_;
  // weight for time regularization term
  double rhoT_, rhoT_track_, rhoT_land_;
  double rhoT_land_origin_;
  double rhoVt_, rhoAt_, rhoPt_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double dv_max_horiz_track_, dv_max_horiz_land_, v_max_vert_track_,
      v_max_vert_land_;
  double dyaw_max_;
  double rhoP_, rhoV_, rhoA_, rhoWP_, rhoRotFactor_;
  double rhoYaw_, rhoDyaw_;
  double rhoV_tail_ = 0.0, rhoA_tail_ = 0.0;
  double rhoTheta_, rhoDtheta_;
  double rhoThrust_, rhoOmega_;
  double rhoPerchingCollision_;
  double rhoPerchingPreception_;
  // theta (arm joint) constraints
  Eigen::Vector3d theta_min_, theta_max_;
  Eigen::Vector3d dtheta_max_;
  // deform parameters
  double rhoDeformDis_, rhoDeformP_, rhoDeformVisibility_, rhoDeformAngle_;
  double rhoDeformConsistAngle_;
  Eigen::Vector3d cur_tar_, last_ego_p_, last_tar_p_;
  // tracking parameters
  double rhoTrackingDis_, rhoTrackingVisibility_, rhoTrackingAngle_;
  double tracking_dur_;
  double tracking_dist_, tolerance_tracking_d_;
  double track_angle_expect_;
  double tracking_dt_;
  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<double> tracking_yaws_;
  // landing parameters
  double v_plus_, robot_l_, robot_r_, platform_r_;
  double preception_d_max_, preception_d_min_;
  bool with_perception_ = false;
  bool short_mode_with_perception_ = false;
  bool short_mode_ = false;
  double short_mode_time_;
  double land_z_min_, land_z_max_;
  double land_z_down_relative_, land_z_up_relative_;
  double eps_pz_;
  // camera parameters
  Eigen::Matrix3d cam2body_R_down_, cam2body_R_front_;
  Eigen::Vector3d cam2body_p_down_, cam2body_p_front_;
  double fx_down_, fy_down_, cx_down_, cy_down_;
  double fx_front_, fy_front_, cx_front_, cy_front_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // flatness map
  flatness::FlatnessMap flatmap_;
  // MINCO Optimizer
  minco::MINCO_S4 minco_s4_opt_;
  minco::MINCO_S4_Uniform minco_s4u_opt_;
  minco::MINCO_S3 minco_s3_opt_;
  minco::MINCO_S2 minco_s2_yaw_opt_;
  minco::MINCO_S3_Uniform minco_s3u_opt_;
  Eigen::MatrixXd initS_, finalS_;
  Eigen::MatrixXd init_yaw_, final_yaw_;
  Eigen::MatrixXd init_arm_ang_, final_arm_ang_;
  // weight for each vertex
  Eigen::MatrixXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double *x_;
  double sum_T_;
  // opt value
  int max_iter_;
  TimePoint opt_start_t_;
  int iter_times_;
  bool is_warm_opt_ = false;

  // IRL evaluation mode: keep the first stage-1 solution and reuse it as the
  // initial point for later stage-2-only solves under the same start/goal/guide.
  bool minco_warm_start_cache_enable_ = false;
  bool minco_warm_start_cache_ready_ = false;
  bool minco_warm_start_cache_force_reuse_ = false;
  double minco_warm_start_cache_tol_ = 1e-5;
  int cached_warm_start_N_ = 0;
  int cached_warm_start_dim_t_ = 0;
  int cached_warm_start_dim_p_ = 0;
  int cached_warm_start_n_thetas_ = 0;
  std::vector<double> cached_warm_start_x_;
  Eigen::MatrixXd cached_warm_start_initS_;
  Eigen::MatrixXd cached_warm_start_finalS_;
  Eigen::MatrixXd cached_warm_start_init_yaw_;
  Eigen::MatrixXd cached_warm_start_final_yaw_;
  Eigen::MatrixXd cached_warm_start_init_thetas_;
  Eigen::MatrixXd cached_warm_start_final_thetas_;
  std::vector<Eigen::Vector3d> cached_warm_start_path_;

  // IRL-only MINCO inner-loop debug. Disabled by default and enabled from the
  // airgrasp_minco_irl launch files when we need to diagnose optimizer shape.
  bool irl_minco_debug_enable_ = false;
  int irl_minco_debug_print_stride_ = 2;
  int irl_minco_debug_marker_stride_ = 2;
  int irl_minco_debug_max_markers_ = 80;
  double irl_minco_debug_sample_dt_ = 0.06;
  std::string irl_minco_debug_stage_ = "idle";
  int irl_minco_debug_marker_seq_ = 0;

  // RC_SDF
  Eigen::MatrixXd aabb_pts_;
  std::vector<Eigen::Vector3d> aabb_pts_vec_;
  std::vector<Eigen::Vector3d> aabb_pts_group1_vec_;
  std::vector<Eigen::Vector3d> aabb_pts_group2_vec_;
  std::vector<Eigen::Vector3d> astar_tracking_wps_;
  double dist_threshold_;
  double dist_threshold_end_;
  double dist_threshold_end_range_;
  Eigen::Vector3d end_pos_goal_;
  Eigen::VectorXd cost_dthetas_rec_;
  Eigen::VectorXd cost_thetas_rec_;

  // IRL sphere collision model
  bool enable_irl_sphere_collision_ = false;
  bool enable_irl_analytic_cylinder_collision_ = false;
  double irl_uam_sphere_radius_ = 0.25;
  double irl_safety_distance_1_ = 0.07;
  double irl_safety_distance_2_ = 0.07;
  double irl_collision_weight_scale_ = 1.0;
  double irl_obs_optimization_margin_ = 0.0;
  Eigen::Vector2d irl_cylinder1_center_xy_ = Eigen::Vector2d(-1.25, 0.0);
  Eigen::Vector2d irl_cylinder2_center_xy_ = Eigen::Vector2d(1.25, 0.0);
  double irl_cylinder_radius_ = 0.30;
  double irl_cylinder1_radius_ = 0.30;
  double irl_cylinder2_radius_ = 0.30;
  double irl_cylinder_z_min_ = 0.0;
  double irl_cylinder_z_max_ = 2.0;
  std::mutex irl_sphere_collision_mutex_;

  // A* path tracking cost for keeping MINCO near the front-end route.
  bool enable_astar_waypoint_tracking_ = false;
  double rhoAstarWaypoint_ = 0.0;
  double rhoAstarWaypointStage2_ = 0.0;

  // arm
  int n_thetas_;
  std::vector<minco::MINCO_S2> minco_s2_theta_opt_vec_;
  Eigen::MatrixXd init_thetas_, final_thetas_;

  // tail constrain
  Eigen::Vector3d p_end_tail_, p_body_tail_, p_end_end_;
  Eigen::Quaterniond q_end_end_, q_body_tail_;
  Eigen::VectorXd thetas_tail_;
  bool en_tail_constraint_ = false;
  double tail_offset_ = 0.1;
  double v_max_tail_ = 0.0;
  double a_max_tail_ = 0.0;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_debug_opt_;
  ros::Publisher pub_irl_minco_debug_markers_;

private:
  void setBoundConds(const Eigen::MatrixXd &iniState,
                     const Eigen::MatrixXd &finState);
  void append_tail_constraint_mid_points(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &final_yaw, const Eigen::MatrixXd &final_thetas,
      std::vector<Eigen::Vector3d> &mid_q_vec, int &N_ret);
  void append_ring_mid_points(const Eigen::MatrixXd &iniState,
                              std::vector<Eigen::Vector3d> &mid_q_vec,
                              double offset = 0.3);

  bool warm_start_cache_matches(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
      const Eigen::MatrixXd &init_thetas,
      const Eigen::MatrixXd &final_thetas,
      const std::vector<Eigen::Vector3d> &ego_path) const;
  void store_warm_start_cache(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
      const Eigen::MatrixXd &init_thetas,
      const Eigen::MatrixXd &final_thetas,
      const std::vector<Eigen::Vector3d> &ego_path, const int opt_dim);

  bool grad_cost_v(const Eigen::Vector3d &v, Eigen::Vector3d &gradv,
                   double &costv);
  bool grad_cost_v(const Eigen::Vector3d &v, const double &v_max,
                   Eigen::Vector3d &gradv, double &costv);
  bool grad_cost_v(const Eigen::Vector3d &v, const double &v_max,
                   const double &rhoV_weight, Eigen::Vector3d &gradv,
                   double &costv);
  // bool grad_cost_v(const Eigen::Vector3d &v, const double &v_max_horiz,
  //                  const double &v_max_vert, Eigen::Vector3d &gradv,
  //                  double &costv);

  bool grad_cost_a(const Eigen::Vector3d &a, Eigen::Vector3d &grada,
                   double &costa);
  bool grad_cost_a(const Eigen::Vector3d &a, const double &a_max,
                   Eigen::Vector3d &grada, double &costa);
  bool grad_cost_a(const Eigen::Vector3d &a, const double &a_max,
                   const double &rhoA_weight, Eigen::Vector3d &grada,
                   double &costa);

  bool grad_cost_yaw(const double &yaw, const Eigen::Vector3d &p,
                     const Eigen::Vector3d &target_p, double &grad_yaw,
                     double &cost_yaw);

  bool grad_cost_yaw_forward(const double &yaw, const Eigen::Vector3d &vel,
                             double &grad_yaw, Eigen::Vector3d &grad_vel,
                             double &cost_yaw);

  bool grad_cost_dyaw(const double &dyaw, double &grad_dyaw, double &cost_dyaw);

  bool grad_cost_theta(const Eigen::VectorXd &thetas,
                       Eigen::VectorXd &grad_thetas, double &cost_thetas);
  bool grad_cost_dtheta(const Eigen::VectorXd &dthetas,
                        Eigen::VectorXd &grad_dthetas, double &cost_dthetas);

  bool grad_cost_thrust(const Eigen::Vector3d &a, Eigen::Vector3d &grada,
                        double &costa);
  bool grad_cost_omega(const Eigen::Vector3d &a, const Eigen::Vector3d &j,
                       Eigen::Vector3d &grada, Eigen::Vector3d &gradj,
                       double &cost);
  bool grad_cost_rate(const Eigen::Vector3d &omg, Eigen::Vector3d &gradomg,
                      double &cost);
  bool grad_cost_omega_yaw(const Eigen::Vector3d &a, const Eigen::Vector3d &j,
                           Eigen::Vector3d &grada, Eigen::Vector3d &gradj,
                           double &cost);
  bool grad_cost_floor(const Eigen::Vector3d &p, Eigen::Vector3d &gradp,
                       double &costp);
  bool grad_cost_ceil(const Eigen::Vector3d &p, Eigen::Vector3d &gradp,
                      double &costp);

  bool grad_cost_perching_collision(const Eigen::Vector3d &pos,
                                    const Eigen::Vector3d &acc,
                                    const Eigen::Vector3d &car_p,
                                    Eigen::Vector3d &gradp,
                                    Eigen::Vector3d &grada,
                                    Eigen::Vector3d &grad_car_p, double &cost);
  bool grad_cost_preception(const Eigen::Vector3d &pos,
                            const Eigen::Vector3d &acc,
                            const Eigen::Vector3d &car_p,
                            Eigen::Vector3d &gradp, Eigen::Vector3d &grada,
                            Eigen::Vector3d &grad_car_p, double &cost);
  bool grad_cost_tracking_p(const Eigen::Vector3d &p,
                            const Eigen::Vector3d &target_p,
                            Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_tracking_angle(const Eigen::Vector3d &p,
                                const Eigen::Vector3d &target_p,
                                Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_tracking_visibility(const Eigen::Vector3d &p,
                                     const Eigen::Vector3d &target_p,
                                     Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_deform_dis(const Eigen::Vector3d &p,
                            const Eigen::Vector3d &target_p,
                            Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_deform_angle(const Eigen::Vector3d &p,
                              const Eigen::Vector3d &target_p,
                              Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_deform_visibility(const Eigen::Vector3d &p,
                                   const Eigen::Vector3d &target_p,
                                   Eigen::Vector3d &gradp, double &costp);

  Eigen::Matrix3d Skew(const Eigen::Vector3d &v);

  inline Eigen::Matrix3d skew_mat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
  }

  // se3
  // Eigen::Matrix3d getQuatTransDX(const Eigen::Vector4d &quat);
  // Eigen::Matrix3d getQuatTransDY(const Eigen::Vector4d &quat);
  // Eigen::Matrix3d getQuatTransDZ(const Eigen::Vector4d &quat);
  // Eigen::Matrix3d getQuatTransDW(const Eigen::Vector4d &quat);

  void calcRotationMatrixGrad(const Eigen::Matrix3d &R_e,
                              const Eigen::Matrix3d &R_goal,
                              const Eigen::Matrix3d &R_drone,
                              const Eigen::VectorXd &thetas, double weight_rot,
                              double &cost_rot, Eigen::VectorXd &grad_thetas);

public:
  TrajOpt(ros::NodeHandle &nh,
          std::shared_ptr<parameter_server::ParaeterSerer> &paraPtr);
  ~TrajOpt() {}

  void set_irl_sphere_safety_distances(const double &d1, const double &d2,
                                       const double &sphere_radius);

  Eigen::Vector3d quatGradToAxisGradient(const Eigen::Quaterniond &q,
                                         const Eigen::Vector4d &grad_q);

  bool grad_cost_collision(const Eigen::Vector3d &p, Eigen::Vector3d &gradp,
                           double &costp);

  bool grad_cost_collision_rc(const Eigen::Vector3d &p,
                              const Eigen::Vector3d &obs,
                              Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_sphere_collision(
      const Eigen::Vector3d &p,
      const std::vector<Eigen::Vector3d> &obs_pts,
      const double &safety_distance, const double &sphere_radius,
      Eigen::Vector3d &gradp, double &costp);

  bool grad_cost_analytic_cylinder_collision(
      const Eigen::Vector3d &p, const Eigen::Vector2d &center_xy,
      const double &cylinder_radius, const double &z_min,
      const double &z_max, const double &safety_distance,
      const double &sphere_radius, Eigen::Vector3d &gradp, double &costp);

  void set_astar_tracking_path(const std::vector<Eigen::Vector3d> &path);

  bool grad_cost_astar_waypoint_tracking(const Eigen::Vector3d &p,
                                         Eigen::Vector3d &gradp,
                                         double &costp);

  bool is_irl_sphere_traj_valid(const Trajectory<7> &traj,
                                const double sample_dt = 0.01,
                                double *min_margin = nullptr,
                                Eigen::Vector3d *min_margin_pos = nullptr,
                                int *min_margin_group = nullptr);

  bool grad_sdf_full_state(
      const Eigen::Vector3d &pt_obs, const Eigen::Vector3d &pos_ego,
      const Eigen::Vector4d &quat, const Eigen::VectorXd &thetas,
      Eigen::Vector3d &gradp, Eigen::Vector4d &grad_quat, Eigen::VectorXd &grad_thetas, double &cost_sdf);

  bool grad_sdf_full_state_jr(
      const Eigen::Vector3d &pt_obs, const Eigen::Vector3d &pos_ego,
      const Eigen::Vector4d &quat, const Eigen::VectorXd &thetas,
      const double &yaw, Eigen::Vector3d &gradp,
      Eigen::Vector4d &grad_quat, // 0 -> w, 1 -> x, 2 -> y, 3 -> z
      double &grad_yaw, Eigen::VectorXd &grad_thetas, double &cost_sdf);

  bool grad_end_pose(const Eigen::Vector3d &p_end_goal,
                     const Eigen::Quaterniond &q_end_goal,
                     const Eigen::Vector3d &p_body_goal,
                     const Eigen::Vector3d &pos_ego,
                     const Eigen::Vector4d &quat, const Eigen::VectorXd &thetas,
                     const double &yaw, Eigen::Vector3d &p_end,
                     Eigen::Vector3d &gradp, Eigen::Vector4d &grad_quat,
                     Eigen::VectorXd &grad_thetas, double &cost);
  bool grad_cost_full_state(const Eigen::Vector3d &pos_ego,
                            const Eigen::Vector4d &quat,
                            const Eigen::VectorXd &thetas,
                            const Eigen::Vector3d &ref_pos,
                            const Eigen::Vector4d &ref_quat,
                            const Eigen::VectorXd &ref_thetas,
                            Eigen::Vector3d &gradp,
                            Eigen::Vector4d &grad_quat,
                            Eigen::VectorXd &grad_thetas,
                            double &cost);

  bool isOccupied_se3(const TrajData &traj_data, const double &cur_t);

  inline void
  set_gridmap_ptr(std::shared_ptr<map_interface::MapInterface> gridmapPtr) {
    gridmapPtr_ = gridmapPtr;
  }

  inline void
  set_rcsdf_ptr(std::shared_ptr<clutter_hand::CH_RC_SDF> rc_sdf_ptr) {
    rc_sdf_ptr_ = rc_sdf_ptr;
    n_thetas_ = rc_sdf_ptr_->getBoxNum() - 1;
    for (size_t i = 0; i < n_thetas_; i++) {
      minco::MINCO_S2 minco_s2_theta_opt;
      minco_s2_theta_opt_vec_.push_back(minco_s2_theta_opt);
    }
  }

  inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface> visPtr) {
    visPtr_ = visPtr;
    visPtr_->cam_vis_down_.set_para(cam2body_R_down_, cam2body_p_down_,
                                    fx_down_, fy_down_, cx_down_, cy_down_);
    visPtr_->cam_vis_front_.set_para(cam2body_R_front_, cam2body_p_front_,
                                     fx_front_, fy_front_, cx_front_,
                                     cy_front_);
  }

  inline void set_with_perception(const bool &is_with_perception) {
    with_perception_ = is_with_perception;
  }

  inline void set_rho_T_land(const double &rhoT_land) {
    rhoT_land_ = rhoT_land;
  }

  inline void set_warm_opt(const bool enable) { is_warm_opt_ = enable; }

  int optimize(const double &delta = 1e-4);

  template <class VEC>
  void extract_mid_pts_from_apath(const std::vector<VEC> &ego_path,
                                  const double &seg_per_dis,
                                  std::vector<VEC> &mid_q_vec, int &N_ret) {
    bool is3D = ego_path[0].size() == 3;

    int N = 0;
    double dis_sum = 0.0, dis_sum_cur = 0.0;
    std::vector<VEC> mid_Q_vec;
    VEC last_p = ego_path.front();
    VEC last_mid_p = ego_path.front();
    mid_Q_vec.push_back(ego_path.front());

    INFO_MSG("extract start");
    cout << "ego_path.size = " << ego_path.size() << endl;

    for (size_t i = 0; i < ego_path.size(); i++) {
      VEC p = ego_path[i];

      INFO_MSG("ego_path[" << i << "]:" << p.transpose());

      bool ray_vaild = gridmapPtr_->checkRayValid(last_mid_p, p);

      if (!ray_vaild &&
          (last_mid_p - last_p).norm() > 1e-3 /*防止前端有屎出现一样两点*/) {
        mid_Q_vec.push_back(last_p);
        last_mid_p = last_p;
      }
      last_p = p;
    }
    mid_Q_vec.push_back(ego_path.back());

    for (size_t i = 0; i < mid_Q_vec.size() - 1; i++) {
      VEC ps = mid_Q_vec[i];
      VEC pt = mid_Q_vec[i + 1];

      INFO_MSG("-------------");
      INFO_MSG("mid_Q_vec[" << i << "]:" << ps.transpose());
      INFO_MSG("mid_Q_vec[" << i + 1 << "]:" << pt.transpose());

      if (!is3D) {
        double error_angle_yaw = rot_util::error_angle(ps(3), pt(3));

        pt(3) = ps(3) + error_angle_yaw;
        INFO_MSG("error_angle_yaw = " << error_angle_yaw
                                      << ", pt(3) = " << pt(3));
      }

      if (i != 0) {
        mid_q_vec.push_back(ps);
      }
      // 还是纯使用pose的距离
      int n = round((ps - pt).head(3).norm() / seg_per_dis);
      if (n > 6)
        n = 6;
      if (n > 1) {
        VEC dir = (pt - ps).normalized();
        double dis_thres = (ps - pt).norm() / n;
        for (int k = 1; k < n; k++) {
          VEC pk = ps + dir * dis_thres * k;
          mid_q_vec.push_back(pk);
        }
      }
    }

    N = mid_q_vec.size() + 1;

    int cnt = 0;
    for (auto p : ego_path)
      cnt++;

    if (N <= 1) { // 万一只有一段
      N_ret = 2;
      VEC p = 0.5 * (ego_path.front() + ego_path.back());

      if (!is3D) {
        double yaw0 = ego_path.front()(3);
        double yaw1 = ego_path.back()(3);
        double error_angle_yaw = rot_util::error_angle(yaw0, yaw1);
        yaw1 = yaw0 + error_angle_yaw;
        p(3) = 0.5 * (yaw0 + yaw1);
      }

      mid_q_vec.push_back(p);
    } else {
      N_ret = N;
    }

    INFO_MSG("extract end N_ret = " << N_ret);
    cnt = 0;
    for (auto p : mid_q_vec) {
      INFO_MSG("mid_q_vec[" << cnt << "]:" << p.transpose());
      cnt++;
    }
  }

  // void extract_mid_pts_from_apath(const std::vector<Eigen::Vector3d>&
  // ego_path,
  //                               const double& seg_per_dis,
  //                               std::vector<Eigen::Vector3d>& mid_q_vec,
  //                               int& N_ret);

  void get_init_taj(const int &N, const Eigen::MatrixXd &init_P_vec,
                    Eigen::VectorXd &init_T_vec);
  void get_init_s4_taj(const int &N, const Eigen::MatrixXd &init_P_vec,
                       Eigen::VectorXd &init_T_vec);
  // goal traj generation
  bool generate_traj(const Eigen::MatrixXd &iniState,
                     const Eigen::MatrixXd &finState, const double &seg_per_dis,
                     const std::vector<Eigen::Vector3d> &ego_path,
                     Trajectory<5> &traj);
  // goal-yaw traj generation
  bool generate_traj(const Eigen::MatrixXd &iniState,
                     const Eigen::MatrixXd &finState,
                     const Eigen::MatrixXd &init_yaw,
                     const Eigen::MatrixXd &final_yaw,
                     const double &seg_per_dis,
                     const std::vector<Eigen::Vector3d> &ego_path,
                     Trajectory<5> &traj);

  // goal yaw
  bool generate_traj(const Eigen::MatrixXd &iniState,
                     const Eigen::MatrixXd &finState,
                     const Eigen::MatrixXd &init_yaw,
                     const Eigen::MatrixXd &final_yaw,
                     const double &seg_per_dis,
                     const std::vector<Eigen::Vector3d> &ego_path,
                     Trajectory<7> &traj);

  // grab traj generation
  bool generate_traj_clutter(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
      const Eigen::MatrixXd &init_thetas, const Eigen::MatrixXd &final_thetas,
      const double &seg_per_dis, const std::vector<Eigen::Vector3d> &ego_path,
      Trajectory<7> &traj);

  bool generate_traj_clutter(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &init_yaw, const Eigen::MatrixXd &final_yaw,
      const Eigen::MatrixXd &init_thetas, const Eigen::MatrixXd &final_thetas,
      const double &seg_per_dis, const std::vector<Eigen::VectorXd> &ego_pathXd,
      Trajectory<7> &traj);

  bool visible_path_deform(const Eigen::Vector3d &ego_p,
                           const std::vector<Eigen::Vector3d> &target_predcit,
                           std::vector<Eigen::Vector3d> &viewpoint_path);

  bool generate_viewpoint(const Eigen::Vector3d &cur_tar,
                          const Eigen::Vector3d &last_ego_p,
                          const Eigen::Vector3d &last_tar_p,
                          Eigen::Vector3d &viewpoint);

  void addTimeIntPenaltyGoal(double &cost);
  void addTimeIntPenaltyTracking(double &cost);
  void addTimeIntPenaltyLanding(double &cost);

  void addTimeCostForward(double &cost);
  void addTimeCostTracking(double &cost);
  void addDeformCost(const Eigen::Vector3d &p, const Eigen::Vector3d &target_p,
                     Eigen::Vector3d &gradp, double &costp);

  Eigen::MatrixXd cal_timebase_jerk(const int order, const double &t);
  Eigen::MatrixXd cal_timebase_acc(const int order, const double &t);
  Eigen::MatrixXd cal_timebase_snap(const int order, const double &t);

  void getJacobian(const Eigen::Vector3d &p, const Eigen::Quaterniond &q,
                   Eigen::MatrixXd &Jacobian);

  // sample trajectory (WITHYAWANDTHETA) into sequences of position / body
  // quaternion / arm joint angles
  void sample_traj_states(const Trajectory<7> &traj,
                          std::vector<Eigen::Vector3d> &pos_seq,
                          std::vector<Eigen::Quaterniond> &q_b2w_seq,
                          std::vector<Eigen::VectorXd> &arm_angles_seq);

  bool irl_minco_debug_build_traj_from_x(const double *x, Trajectory<7> &traj);
  void irl_minco_debug_reset_markers();
  void irl_minco_debug_snapshot_from_x(const double *x, double fx, double step,
                                       int lbfgs_iter, int ls, bool force,
                                       const std::string &status);
  void irl_minco_debug_snapshot_traj(const Trajectory<7> &traj,
                                     const std::string &stage, int lbfgs_iter,
                                     double fx, double step, int ls,
                                     bool force,
                                     const std::string &status);


  bool feasibleCheck(Trajectory<7> &traj);

  bool check_collilsion(const Eigen::Vector3d &pos, const Eigen::Vector3d &acc,
                        const Eigen::Vector3d &car_p);

  inline void clear_cost_rec() {
    cost_snap_rec_ = 0.0;
    cost_v_rec_ = 0.0;
    cost_a_rec_ = 0.0;
    cost_thrust_rec_ = 0.0;
    cost_omega_rec_ = 0.0;
    cost_perching_collision_rec_ = 0.0;
    cost_perching_precep_rec_ = 0.0;
    cost_t_rec_ = 0.0;
    deltaT_rec_ = 0.0;
    cost_tracking_dis_rec_ = 0.0;
    cost_collision_rec_ = 0.0;
    cost_tracking_ang_rec_ = 0.0;
    cost_tracking_vis_rec_ = 0.0;
    cost_yaw_rec_ = 0.0;
    cost_dyaw_rec_ = 0.0;
    cost_wp_rec_ = 0.0;

    cost_dthetas_rec_ = Eigen::VectorXd::Zero(minco_s2_theta_opt_vec_.size());
    cost_thetas_rec_ = Eigen::VectorXd::Zero(minco_s2_theta_opt_vec_.size());
  }
  inline double get_snap_cost() { return cost_snap_rec_; }
  inline double get_v_cost() { return cost_v_rec_; }
  inline double get_a_cost() { return cost_a_rec_; }
  inline double get_thrust_cost() { return cost_thrust_rec_; }
  inline double get_omega_cost() { return cost_omega_rec_; }
  inline double get_perching_collision_cost() {
    return cost_perching_collision_rec_;
  }
  inline double get_perching_precept_cost() {
    return cost_perching_precep_rec_;
  }
  inline double get_t_cost() { return cost_t_rec_; }
  inline double get_tracking_dis_cost() { return cost_tracking_dis_rec_; }
  inline double get_tracking_ang_cost() { return cost_tracking_ang_rec_; }
  inline double get_tracking_vis_cost() { return cost_tracking_vis_rec_; }
  inline double get_collision_cost() { return cost_collision_rec_; }
  inline double get_yaw_cost() { return cost_yaw_rec_; }
  inline double get_dyaw_cost() { return cost_dyaw_rec_; }

  // inline void dashboard_cost_print(){
  //   // return;
  //   if (cost_lock_.test_and_set()) return;
  //   cost_lock_.clear();
  // }

  inline void dashboard_cost_print() {
    // return;

    if (cost_lock_.test_and_set())
      return;
    // INFO_MSG("-------------------[Traj Opt Cost]----------------");
    // INFO_MSG("Total Cost: " << std::setprecision(10) << cost);
    // INFO_MSG("Snap: " << std::setprecision(10) << get_snap_cost());
    // INFO_MSG("dT(" << deltaT_rec_ << "): " << get_t_cost());
    // INFO_MSG("Vel: " << get_v_cost());
    // INFO_MSG("Acc: " << get_a_cost());
    // INFO_MSG("Pos: " << std::setprecision(10) << get_collision_cost());
    // INFO_MSG("Omega: " << get_omega_cost());
    // INFO_MSG("Thrust: " << get_thrust_cost());
    // INFO_MSG("Yaw: " << get_yaw_cost());
    // INFO_MSG("dYaw: " << get_dyaw_cost());

    quadrotor_msgs::DebugOpt msg;
    msg.cost_snap = cost_snap_rec_;
    msg.cost_v = cost_v_rec_;
    msg.cost_a = cost_a_rec_;
    msg.cost_omega = cost_omega_rec_;
    msg.cost_t = cost_t_rec_;
    msg.cost_collision = cost_collision_rec_;
    msg.cost_yaw = cost_yaw_rec_;
    msg.cost_dyaw = cost_dyaw_rec_;
    msg.cost_wp = cost_wp_rec_;
    for (size_t i = 0; i < cost_dthetas_rec_.size(); i++) {
      msg.cost_dthetas.push_back(cost_dthetas_rec_[i]);
    }
    pub_debug_opt_.publish(msg);

    cost_lock_.clear();
  }

  inline void set_v_max(const double &v_max) { vmax_ = v_max; }
  inline void set_en_through_ring(bool en) { en_through_ring_ = en; }
  inline void set_ring_pose(const geometry_msgs::PoseStamped &pose) {
    ring_pose_ = pose;
    has_ring_pose_ = true;
  }
  inline bool through_gap_enabled() const { return en_through_ring_; }
  inline bool has_ring_pose() const { return has_ring_pose_; }
  inline const geometry_msgs::PoseStamped &ring_pose() const {
    return ring_pose_;
  }

  geometry_msgs::PoseStamped ring_pose_;
  std::vector<Eigen::Vector3d> ring_mid_pts_;
  Eigen::Quaterniond ring_mid_quat_ = Eigen::Quaterniond::Identity();
  bool has_ring_mid_quat_ = false;
  bool has_ring_pose_ = false;
  bool en_through_ring_ = false;
};

inline double objectiveFuncGoal(void *ptrObj, const double *x, double *grad,
                                const int n);
inline int earlyExitGoal(void *ptrObj, const double *x, const double *grad,
                         const double fx, const double xnorm,
                         const double gnorm, const double step, int n, int k,
                         int ls);

inline double objectiveFuncTracking(void *ptrObj, const double *x, double *grad,
                                    const int n);
inline int earlyExitTracking(void *ptrObj, const double *x, const double *grad,
                             const double fx, const double xnorm,
                             const double gnorm, const double step, int n,
                             int k, int ls);

inline double objectiveFuncLanding(void *ptrObj, const double *x, double *grad,
                                   const int n);
inline int earlyExitLanding(void *ptrObj, const double *x, const double *grad,
                            const double fx, const double xnorm,
                            const double gnorm, const double step, int n, int k,
                            int ls);

inline double objectiveFuncDeform(void *ptrObj, const double *x, double *grad,
                                  const int n);

inline int earlyExitDeform(void *ptrObj, const double *x, const double *grad,
                           const double fx, const double xnorm,
                           const double gnorm, const double step, int n, int k,
                           int ls);

double expC2(double t);
double logC2(double T);
double smoothedL1(const double &x, double &grad);
bool smoothedL1(const double &x, const double &mu, double &f, double &df);
double smoothed01(const double &x, double &grad, const double mu = 0.01);
double penF(const double &x, double &grad);
double penF2(const double &x, double &grad);
double gdT2t(double t);
void forwardT(const double &t, double &T);
void backwardT(const double &T, double &t);
void forwardT_sum(const Eigen::Ref<const Eigen::VectorXd> &t, const double &sT,
                  Eigen::Ref<Eigen::VectorXd> vecT);
void backwardT_sum(const Eigen::Ref<const Eigen::VectorXd> &vecT,
                   Eigen::Ref<Eigen::VectorXd> t);
void forwardT(const Eigen::Ref<const Eigen::VectorXd> &t,
              Eigen::Ref<Eigen::VectorXd> vecT);
void backwardT(const Eigen::Ref<const Eigen::VectorXd> &vecT,
               Eigen::Ref<Eigen::VectorXd> t);
void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd> &t,
                   const Eigen::Ref<const Eigen::VectorXd> &gradT,
                   Eigen::Ref<Eigen::VectorXd> gradt);
void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd> &t, const double &sT,
                   const Eigen::Ref<const Eigen::VectorXd> &gradT,
                   Eigen::Ref<Eigen::VectorXd> gradt);

Trajectory<5> getS3TrajWithYaw(minco::MINCO_S3 &mincos3_opt,
                               minco::MINCO_S2 &mincoyaw_opt);
Trajectory<7> getS4UTrajWithYaw(minco::MINCO_S4_Uniform &mincos4u_opt,
                                minco::MINCO_S2 &mincoyaw_opt);
Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4 &mincos4_opt,
                               minco::MINCO_S2 &mincoyaw_opt);
Trajectory<7>
getS4TrajWithYawAndThetas(minco::MINCO_S4 &mincos4_opt,
                          minco::MINCO_S2 &mincoyaw_opt,
                          std::vector<minco::MINCO_S2> &minco_thetas_opt_vec);
} // namespace traj_opt
