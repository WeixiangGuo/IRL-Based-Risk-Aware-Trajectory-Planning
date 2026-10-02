#include "planning/tlplanner.h"

#include <limits>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>
#include <visualization_msgs/Marker.h>

using rot_util = rotation_util::RotUtil;

namespace {
bool parseGuidePathSpec(const std::string& spec,
                        std::vector<Eigen::Vector3d>& path) {
    path.clear();
    std::stringstream point_stream(spec);
    std::string point_token;

    while (std::getline(point_stream, point_token, ';')) {
        if (point_token.empty()) {
            continue;
        }

        std::stringstream value_stream(point_token);
        std::string value_token;
        std::vector<double> values;

        while (std::getline(value_stream, value_token, ',')) {
            if (value_token.empty()) {
                continue;
            }
            try {
                values.push_back(std::stod(value_token));
            } catch (const std::exception&) {
                path.clear();
                return false;
            }
        }

        if (values.size() != 3) {
            path.clear();
            return false;
        }
        path.emplace_back(values[0], values[1], values[2]);
    }

    return path.size() >= 2;
}
}  // namespace

namespace tlplanner{
TLPlanner::TLPlanner(std::shared_ptr<parameter_server::ParaeterSerer>& para_ptr):paraPtr_(para_ptr){
    paraPtr_->get_para("Webvis_hz", web_vis_hz_);   
    paraPtr_->get_para("is_use_viewpoint", is_use_viewpoint_);   
    paraPtr_->get_para("plan_estimated_duration", plan_estimated_duration_);   
    ros::param::param("~clear_traj_robot_markers_before_publish",
                      clear_traj_robot_markers_before_publish_, false);

    paraPtr_->get_para("plan_horizon_len", plan_horizon_len_); 

    int s_guide_path_enable = 0;
    try {
        paraPtr_->get_para("s_guide_path_enable", s_guide_path_enable);
        s_guide_path_enable_ = s_guide_path_enable != 0;
    } catch (const std::exception&) {
        s_guide_path_enable_ = false;
    }
    ros::param::param("~s_guide_path_enable",
                      s_guide_path_enable_,
                      s_guide_path_enable_);
    ros::param::param("~s_guide_bypass_astar",
                      s_guide_bypass_astar_,
                      s_guide_bypass_astar_);

    try {
        paraPtr_->get_para("s_guide_endpoint_tolerance",
                           s_guide_endpoint_tolerance_);
    } catch (const std::exception&) {
    }
    ros::param::param("~s_guide_endpoint_tolerance",
                      s_guide_endpoint_tolerance_,
                      s_guide_endpoint_tolerance_);

    std::string s_guide_path_spec;
    try {
        paraPtr_->get_para("s_guide_path", s_guide_path_spec);
    } catch (const std::exception&) {
    }
    std::string s_guide_path_ros_param;
    if (ros::param::get("~s_guide_path", s_guide_path_ros_param)) {
        s_guide_path_spec = s_guide_path_ros_param;
    }
    if (!s_guide_path_spec.empty()) {
        if (!parseGuidePathSpec(s_guide_path_spec, s_guide_path_)) {
            ROS_WARN("[tlplanner] Failed to parse s_guide_path, disabling guide path.");
            s_guide_path_enable_ = false;
        }
    } else {
        s_guide_path_.clear();
    }
    if (s_guide_bypass_astar_ && !s_guide_path_enable_) {
        ROS_WARN("[tlplanner] s_guide_bypass_astar requested but s_guide_path_enable is false; A* remains active.");
        s_guide_bypass_astar_ = false;
    }

    if (s_guide_path_enable_ && s_guide_path_.size() >= 2) {
        ROS_INFO("[tlplanner] S guide path enabled, points=%zu, endpoint_tol=%.3f, bypass_astar=%s",
                 s_guide_path_.size(),
                 s_guide_endpoint_tolerance_,
                 s_guide_bypass_astar_ ? "true" : "false");
    }
}


TLPlanner::PlanResState TLPlanner::plan_goal(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data){
    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw, init_thetas;
    init_state.setZero(3, 4);
    init_yaw.setZero(1, 2);
    init_thetas.setZero(init_state_in.theta_.size(),2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;
    init_thetas.col(0) = init_state_in.theta_;
    init_thetas.col(1) = init_state_in.dtheta_;

    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("init v: " << init_state.col(1).transpose());
    INFO_MSG("init a: " << init_state.col(2).transpose());
    INFO_MSG("init j: " << init_state.col(3).transpose());
    INFO_MSG("init yaw: " << init_yaw);
    INFO_MSG("init thetas: " << init_thetas.col(0).transpose());
    INFO_MSG("init d_thetas: " << init_thetas.col(1).transpose());
    INFO_MSG("tar p: " << target_data.odom_p_.transpose());
    INFO_MSG("tar v: " << target_data.odom_v_.transpose());
    INFO_MSG("tar a: " << target_data.odom_a_.transpose());
    INFO_MSG("tar thetas:" << target_data.theta_.transpose());
    INFO_MSG("tar d_thetas:" << target_data.dtheta_.transpose());
    // INFO_MSG("tar theta: " << rot_util::quaternion2yaw(target_data.odom_q_));

    //! 4. path search
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path3d, way_pts;
    std::vector<Eigen::VectorXd> pathXd;
    bool generate_new_traj_success;

    std::vector<Eigen::Vector3d> vis_goal_p;
    vis_goal_p.push_back(target_data.odom_p_);
    visPtr_->visualize_pointcloud(vis_goal_p, "front_end_goal");

    const std::vector<Eigen::Vector3d> s_guide_path =
        build_s_guide_path(p_start, target_data.odom_p_);
    if (s_guide_bypass_astar_) {
        if (s_guide_path.size() < 2) {
            INFO_MSG_RED("[tlplanner] S guide bypass requested but guide path is invalid.");
            return FAIL;
        }
        path3d = s_guide_path;
        visPtr_->visualize_path(path3d, "s_guide");
        generate_new_traj_success = true;
        INFO_MSG_GREEN_BG("----- Skip A*: use fixed S guide path");
        INFO_MSG_GREEN("[tlplanner] use S guide path, points=" << path3d.size());
    } else {
        INFO_MSG_GREEN_BG("----- Start search path");
        generate_new_traj_success = envPtr_->short_astar(p_start, target_data.odom_p_, path3d); // get a path3d from current pose to target pose
        if (path3d.empty() || (path3d.front() - p_start).norm() > 1e-6)
            path3d.insert(path3d.begin(), p_start);
        if (path3d.empty() || (path3d.back() - target_data.odom_p_).norm() > 1e-6)
            path3d.push_back(target_data.odom_p_);
        if(generate_new_traj_success)
            visPtr_->visualize_path(path3d, "astar");
        else{
            INFO_MSG_RED("[tlplanner] search path3d fail!");
            return FAIL;
        }

        if (s_guide_path.size() >= 2) {
            path3d = s_guide_path;
            visPtr_->visualize_path(path3d, "s_guide");
            INFO_MSG_GREEN("[tlplanner] use S guide path, points=" << path3d.size());
        }
    }

    //! 5. traj opt
    Trajectory<7> traj_goal;
    Eigen::MatrixXd final_state, final_yaw, final_thetas;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);
    final_thetas.setZero(target_data.theta_.size(),2);
    final_state.col(0) = path3d.back();
    final_state.col(1) = target_data.odom_v_;
    final_yaw(0, 0) = rot_util::quaternion2yaw(target_data.odom_q_);
    final_thetas.col(0) = target_data.theta_;
    // final_thetas.col(1) = target_data.dtheta_;
    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("end p: " << final_state.col(0).transpose());
    INFO_MSG("init yaw: " << init_yaw(0, 0));
    INFO_MSG("end yaw: " << final_yaw(0, 0));

    // pass ring pose to trajopt if enabled
    if (en_through_ring_ && trajoptPtr_){
        trajoptPtr_->set_en_through_ring(true);
        if (has_ring_pose_){
            trajoptPtr_->set_ring_pose(ring_pose_);
        }
    }else if(trajoptPtr_){
        trajoptPtr_->set_en_through_ring(false);
    }

    // 3D
    generate_new_traj_success = trajoptPtr_->generate_traj_clutter(init_state, final_state, init_yaw, 
                                                                    final_yaw, init_thetas, final_thetas,
                                                                    0.8, path3d, traj_goal); 
    
    // XD
    // generate_new_traj_success = trajoptPtr_->generate_traj_clutter(init_state, final_state, init_yaw, 
    //                                                                 final_yaw, init_thetas, final_thetas,
    //                                                                 2.0, pathXd, traj_goal); 

    auto publish_full_state_seq = [&](const Trajectory<7>& traj,
                                      const std::string& topic,
                                      const visualization_rc_sdf::Color& color){
        if (!rc_sdf_ptr_ || !trajoptPtr_) return;
        std::vector<Eigen::Vector3d> pos_seq;
        std::vector<Eigen::Quaterniond> q_seq;
        std::vector<Eigen::VectorXd> arm_seq;
        trajoptPtr_->sample_traj_states(traj, pos_seq, q_seq, arm_seq);

        visualization_msgs::MarkerArray marker_array;
        if (clear_traj_robot_markers_before_publish_) {
            visualization_msgs::Marker clear_marker;
            clear_marker.header.frame_id = "world";
            clear_marker.header.stamp = ros::Time::now();
            clear_marker.ns = "shapes";
            clear_marker.id = 0;
            clear_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);
        }
        for(size_t i = 0; i < pos_seq.size(); ++i){
            rc_sdf_ptr_->getRobotMarkerArray(pos_seq[i], q_seq[i], arm_seq[i],
                                             marker_array, color, 0.25);
        }
        rc_sdf_ptr_->robotMarkersPub(marker_array, topic);
    };

    // TODO add vis seq
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        visPtr_->visualize_traj(traj_goal, "traj_failed");
        publish_full_state_seq(traj_goal, "traj_failed_robot_markers",
                               visualization_rc_sdf::Color::red);
        return FAIL;
    }
    traj_data.traj_d7_ = traj_goal;
    traj_data.state_ = TrajData::D7;

    if(web_vis_hz_ > 0){
        visPtr_->visualize_traj(traj_goal, "traj");
        publish_full_state_seq(traj_goal, "traj_robot_markers",
                               visualization_rc_sdf::Color::green);
    }

    return PLANSUCC;
}

std::vector<Eigen::Vector3d> TLPlanner::build_s_guide_path(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) const {
    std::vector<Eigen::Vector3d> guide_path;
    if (!s_guide_path_enable_ || s_guide_path_.size() < 2) {
        return guide_path;
    }

    std::vector<Eigen::Vector3d> templated_path = s_guide_path_;
    templated_path.back() = goal;

    double best_dist_sq = std::numeric_limits<double>::infinity();
    size_t best_segment_idx = 0;
    double best_segment_s = 0.0;

    for (size_t i = 0; i + 1 < templated_path.size(); ++i) {
        const Eigen::Vector3d& a = templated_path[i];
        const Eigen::Vector3d& b = templated_path[i + 1];
        const Eigen::Vector3d ab = b - a;
        const double ab_sq_norm = ab.squaredNorm();

        double s = 0.0;
        if (ab_sq_norm > 1e-8) {
            s = (start - a).dot(ab) / ab_sq_norm;
            if (s < 0.0) {
                s = 0.0;
            } else if (s > 1.0) {
                s = 1.0;
            }
        }

        const Eigen::Vector3d ref = a + s * ab;
        const double dist_sq = (start - ref).squaredNorm();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_segment_idx = i;
            best_segment_s = s;
        }
    }

    guide_path.push_back(start);
    if (best_dist_sq <= s_guide_endpoint_tolerance_ * s_guide_endpoint_tolerance_) {
        for (size_t i = best_segment_idx + 1; i < templated_path.size(); ++i) {
            if ((templated_path[i] - guide_path.back()).norm() > 1e-3) {
                guide_path.push_back(templated_path[i]);
            }
        }
    } else {
        for (size_t i = 1; i < templated_path.size(); ++i) {
            if ((templated_path[i] - guide_path.back()).norm() > 1e-3) {
                guide_path.push_back(templated_path[i]);
            }
        }
        ROS_WARN("[tlplanner] start is %.3f m away from S guide path, using full guide from current start.",
                 std::sqrt(best_dist_sq));
    }

    if ((guide_path.back() - goal).norm() > 1e-6) {
        guide_path.push_back(goal);
    }
    (void)best_segment_s;
    return guide_path;
}



bool TLPlanner::valid_cheack(const TrajData& traj_data, const TimePoint& cur_t){

    double t0 = durationSecond(cur_t, traj_data.start_time_); //返回轨迹开始时间到当前时间的时间差
    t0 = t0 > 0.0 ? t0 : 0.0;
    INFO_MSG("------------valid check");
    if (traj_data.state_ == TrajData::D7 && trajoptPtr_){
        double min_irl_margin = 0.0;
        Eigen::Vector3d min_irl_margin_pos = Eigen::Vector3d::Zero();
        int min_irl_margin_group = 0;
        if (!trajoptPtr_->is_irl_sphere_traj_valid(
                traj_data.traj_d7_, 0.01, &min_irl_margin,
                &min_irl_margin_pos, &min_irl_margin_group)){
            INFO_MSG_RED("traj is invalid, IRL sphere hits obstacle group "
                         << min_irl_margin_group << ", margin="
                         << min_irl_margin << ", pos="
                         << min_irl_margin_pos.transpose());
            return false;
        }
    }
    for (double t = t0; t < traj_data.getTotalDuration(); t += 0.01){ //遍历轨迹            
        // #ifndef USE_RC_SDF
        //     Eigen::Vector3d p = traj_data.getPos(t);
        //     if (gridmapPtr_->isOccupied(p)){
        //         INFO_MSG_RED("traj is invalid, hit obstacle");
        //         return false;
        //     }
        // #else
        //     if(trajoptPtr_->isOccupied_se3(traj_data,t)){
        //         INFO_MSG_RED("traj is invalid, hit obstacle");
        //         return false;
        //     }
        // #endif

        Eigen::Vector3d p = traj_data.getPos(t);
        if (gridmapPtr_->isOccupied(p)){
            INFO_MSG_RED("traj is invalid, hit obstacle");
            return false;
        }

    }

    return true;
}


void TLPlanner::cal_local_goal_from_path(std::vector<Eigen::Vector3d>& path){

    // 在外部函数中定义内部函数
    std::vector<Eigen::Vector3d> path_temp;

    Eigen::Vector3d p_a, p_b, p_o;
    for(int i = 0; i < path.size(); i++){
        if(i == 0){
            path_temp.push_back(path[i]);
        }else{
            if((path[i] - path_temp.back()).norm() < plan_horizon_len_){
                path_temp.push_back(path[i]);
            }
            else {
                p_a = path_temp.back();
                p_b = path[i];

                p_o = path[0];


                // 计算ab线段上距离 path[0] 长度为 plan_horizon_len_ 的点
                double cos_a = (p_o - p_a).dot(p_b - p_a) / ((p_o - p_a).norm() * (p_b - p_a).norm());
                // 垂线长度

                double a = 1;
                double b = -2 * cos_a * (p_o - p_a).norm();
                double c = (p_o - p_a).norm() * (p_o - p_a).norm() 
                        - plan_horizon_len_ * plan_horizon_len_;
                
                double t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);

                path_temp.push_back(p_a + t * (p_b - p_a));
                break;
            }
        }
    }

    path = path_temp;
}

} // namespace tlplanner
