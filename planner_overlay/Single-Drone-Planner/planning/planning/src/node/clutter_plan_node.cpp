#include "node/plan_node.h"

Planner::Planner(std::shared_ptr<ShareDataManager> dataManagerPtr,
                 ros::NodeHandle& nh,
                 std::shared_ptr<parameter_server::ParaeterSerer> paraPtr,
                 std::shared_ptr<vis_interface::VisInterface> visPtr):
                 dataManagerPtr_(dataManagerPtr),
                 nh_(nh),
                 paraPtr_(paraPtr),
                 visPtr_(visPtr)
{
  paraPtr_->get_para("plan_hz", plan_hz_);
  paraPtr_->get_para("plan_estimated_duration", plan_estimated_duration_);   
  paraPtr_->get_para("local_horizon", local_horizon_);

  // feed clutter hand parameter
  paraPtr_->get_para("vmax", v_max_);

  rc_sdf_ptr_ = std::make_shared<clutter_hand::CH_RC_SDF>();
  rc_sdf_ptr_->initMap(nh, true, false);

  int goal_method;
  paraPtr_->get_para("goal_method", goal_method);
  goal_method_ = (enum GoalMethod)goal_method;

  trajoptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh, paraPtr_);
  tlplannerPtr_ = std::make_shared<tlplanner::TLPlanner>(paraPtr_);
  
  gridmapPtr_ = std::make_shared<map_interface::MapInterface>(nh_);
  envPtr_ = std::make_shared<env::Env>(paraPtr_, gridmapPtr_);

  INFO_MSG_GREEN("[CH_PLAN] ptr init done");

  trajoptPtr_->set_gridmap_ptr(gridmapPtr_);
  trajoptPtr_->set_rcsdf_ptr(rc_sdf_ptr_);  

  tlplannerPtr_->set_gridmap_ptr(gridmapPtr_);
  tlplannerPtr_->set_env_ptr(envPtr_);
  // tlplannerPtr_->set_sdf_ptr(sdf_map_);
  tlplannerPtr_->set_trajopt_ptr(trajoptPtr_);
  tlplannerPtr_->set_data_ptr(dataManagerPtr_);
  tlplannerPtr_->set_rc_sdf_ptr(rc_sdf_ptr_);

  envPtr_->set_vis_ptr(visPtr_);
  trajoptPtr_->set_vis_ptr(visPtr_);
  tlplannerPtr_->set_vis_ptr(visPtr_);

  std::vector<Eigen::Vector3d> path;
  visPtr_->visualize_path(path, "astar");
  visPtr_->visualize_pointcloud(path, "astar_vp");
  visPtr_->visualize_path(path, "car_predict");
  visPtr_->visualize_pointcloud(path, "mid_waypts");
  visPtr_->visualize_path(path, "rrt_star_final_path");
  visPtr_->visualize_pointcloud(path, "rrt_star_final_wpts");
  visPtr_->visualize_pointcloud(path, "AABB_pts");

  vector<vector<Eigen::Vector3d>> routes;
  visPtr_->visualize_path_list(routes, "rrt_star_paths");

  std::vector<std::pair<Eigen::Vector3d, double>> pcl_i;
  visPtr_->visualize_pointcloud_itensity(pcl_i, "IG_gains");

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> lines;
  visPtr_->visualize_arrows(lines, "debug_grad_collision_p");
  visPtr_->visualize_arrows(lines, "debug_grad_collision_q");

  INFO_MSG_GREEN("[CH_PLAN] vis_ptr init done");

  // getIGPrarm();

  mani_goal_sub_ = nh_.subscribe("/mani_goal", 1, &Planner::mani_goal_cb, this);
  planner_failure_pub_ = nh_.advertise<std_msgs::Header>("planner_failure", 10);

  // airgrasp
  theta_cur_sub_ = nh_.subscribe("joint_state_est", 1, &Planner::theta_cur_cb, this);
  irl_safety_distances_sub_ =
      nh_.subscribe("irl_safety_distances", 1,
                    &Planner::irl_safety_distances_cb, this);
  paraPtr_->get_para("en_through_ring", en_through_ring_);
  tlplannerPtr_->set_en_through_ring(en_through_ring_);
  if (en_through_ring_){
    ring_pose_sub_ = nh_.subscribe("stick_ring_pose", 1, &Planner::ring_pose_cb, this);
  }

  INFO_MSG_GREEN("[plan node] Planner Init Done.");

}

void Planner::irl_safety_distances_cb(const geometry_msgs::Vector3::ConstPtr& msg){
  if (!trajoptPtr_) {
    return;
  }
  trajoptPtr_->set_irl_sphere_safety_distances(msg->x, msg->y, msg->z);
}

void Planner::publish_planner_failure(const std::string& reason){
  std_msgs::Header msg;
  msg.seq = ++planner_failure_seq_;
  msg.stamp = ros::Time::now();
  msg.frame_id = reason;
  planner_failure_pub_.publish(msg);
  ROS_WARN("[plan node] Publish planner failure seq=%u reason=%s", msg.seq, reason.c_str());
}

void Planner::plan_thread(){
  while (!dataManagerPtr_->s_exit_)
  {    
    int thread_dur = (int) (1000.0 / (double) plan_hz_);
    if (plan_hz_ < 0) continue;
    TimePoint t0 = TimeNow();
    PlanMode mode = get_mode();
    INFO_MSG("------------------- Plan Thread(" <<  get_mode_name() <<") -----------------------");
    #ifdef SS_DBUS
    INFO_MSG("[systime] " << TimeNow() / 1e6);
    #endif
    if (mode == IDLE){
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    }

    //! TODO TODO TODO
    //! 1. 获得 当前状态
    Odom odom_data;
    if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data) || !has_theta_cur_){
      set_state(PLANFAIL);
      if (mode == GOAL){
        publish_planner_failure("missing_odom_or_theta");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    } 
    odom_data.theta_ = theta_cur_;
    odom_data.dtheta_ = dtheta_cur_;
    Eigen::Vector3d odom_p = odom_data.odom_p_;
    Eigen::Vector3d odom_v = odom_data.odom_v_;
    // double odom_yaw = rot_util::quaternion2yaw(odom_data.odom_q_);

    if (en_through_ring_ && !has_ring_pose_){
      set_state(PLANFAIL);
      if (mode == GOAL){
        publish_planner_failure("missing_ring_pose");
      }
      std::cout << "waiting for ring pose..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    } 
    if (en_through_ring_ && has_ring_pose_){
      tlplannerPtr_->set_ring_pose(ring_pose_);
    }

    Eigen::Vector3d end_pos_cur;
    Eigen::Quaterniond end_q_cur;
    rc_sdf_ptr_->getCurEndPose(odom_p, odom_data.odom_q_ ,end_pos_cur, end_q_cur);
    // INFO_MSG("[plan node] get end_pos_cur: " << end_pos_cur.transpose());
    // INFO_MSG("[plan node] get end_q_cur: " << end_q_cur.coeffs().transpose());
    // INFO_MSG("[plan node] get odom: " << odom_p.transpose());
    // INFO_MSG("[plan node] get vel: " << odom_v.transpose());
    if (mode == HOVER){
      dataManagerPtr_->save_hover_p(odom_p, TimeNow(), dataManagerPtr_->traj_info_);
      set_state(NOTNEEDPLAN);
      INFO_MSG("[plan node] hovering...");
    }

    //! 2. 获得 地图信息
    // lock until this plan loop finish
    
    //! 3. 获得 目标状态
    Odom goal_data;
    bool get_new_goal = false;
    switch (goal_method_)
    {
      case RVIZ_GOAL: // 通过 rviz 发布目标点
      {
        // 检索 dataManagerPtr_->goal_info_ 修改z坐标
        dataManagerPtr_->get_odom(dataManagerPtr_->goal_info_, goal_data); 
        goal_data.dtheta_ = Eigen::Vector3d::Zero();
        goal_data.theta_ = Eigen::Vector3d::Zero();
        last_goal_data_ = goal_data;
        get_new_goal = true;
        break;
      }

      case FULL_STATE_GOAL: // 通过 rviz 发布目标点
      {
        // 检索 dataManagerPtr_->goal_info_ 修改z坐标
        dataManagerPtr_->get_odom(dataManagerPtr_->goal_info_, goal_data); 
        last_goal_data_ = goal_data;
        get_new_goal = true;
        break;
      }

      case PRE_SEQUENCE: // 网络预测序列
          INFO_MSG("PRE_SEQUENCE not defined yet~~~");
          break;

      default:
        ROS_ERROR("GOAL METHOD ERROR");
        break;
    }

    if (mode == HOVER){
      BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", HOVER));
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    }

    //! 4. 从上次执行轨迹获得初始状态
    TimePoint t00 = TimeNow();
    TimePoint replan_stamp = addDuration(t00, plan_estimated_duration_);
    double replan_t = 0.0;
    Odom init_state;
    TrajData traj_now;
    bool traj_last_valid = has_traj_last_ && 
                           (traj_last_.state_ == TrajData::D5 || traj_last_.state_ == TrajData::D7);
    // get_new_goal = !traj_last_valid && state_cnt_%100==0;

    // 在执行时间内
    if (traj_last_valid){
      replan_t = durationSecond(replan_stamp, traj_last_.start_time_);
      replan_t = replan_t <= traj_last_.getTotalDuration() ? replan_t : traj_last_.getTotalDuration() - 1e-3;
      traj_last_valid = traj_last_valid && (replan_t <= traj_last_.getTotalDuration());
      if (traj_last_valid){
        Eigen::Vector3d traj_pos_now = traj_last_.getPos(replan_t);
        double reset_jump = (traj_pos_now - odom_data.odom_p_).norm();
        if (reset_jump > 0.5){
          traj_last_valid = false;
          ROS_WARN("[plan node] Current odom jumps away from last traj by %.3f m, replan from current odom.", reset_jump);
        }
      }
    }
    // last traj not valid, plan from current odom
    if (!traj_last_valid){
      init_state = odom_data;
      init_state.odom_a_.setZero();
      init_state.odom_dyaw_ = 0.0;

      if (traj_last_.getTrajType() == TrajType::WITHYAWANDTHETA){
        init_state.theta_ = theta_cur_;
        std::cout << "init_state.theta_:\n" << init_state.theta_ << std::endl;
        init_state.dtheta_ = dtheta_cur_;
      }

      ROS_ERROR("last traj not valid, plan from current odom");
      std::cout << "init_p:" << init_state.odom_p_.transpose() << std::endl;
    }
    // plan from last traj
    else{
      init_state.odom_p_ = traj_last_.getPos(replan_t);
      init_state.odom_v_ = traj_last_.getVel(replan_t);
      init_state.odom_a_ = traj_last_.getAcc(replan_t);
      init_state.odom_j_ = traj_last_.getJer(replan_t);
      init_state.odom_q_ = rot_util::yaw2quaternion(traj_last_.getAngle(replan_t));
      init_state.odom_dyaw_ = traj_last_.getAngleRate(replan_t);

      if (traj_last_.getTrajType() == TrajType::NOYAW){
        init_state.odom_q_ = odom_data.odom_q_;
        init_state.odom_dyaw_ = 0.0;
      }

      if (traj_last_.getTrajType() == TrajType::WITHYAWANDTHETA){
        Eigen::VectorXd thetas, dthetas;
        init_state.theta_ = traj_last_.getTheta(replan_t);
        init_state.dtheta_ = traj_last_.getThetaRate(replan_t);
      }

    }
    INFO_MSG("[plan node] get init state");


    //! 5. planner
    tlplanner::TLPlanner::PlanResState plan_res;
    bool no_need_plan = false;
    switch (mode)
    {
    case GOAL:
      {
        // GData<double> track_dis_data, track_h_data;
        // dataManagerPtr_->get_data(dataManagerPtr_->tracking_dis_info_, track_dis_data);
        // dataManagerPtr_->get_data(dataManagerPtr_->tracking_height_info_, track_h_data);
        // tracking_dis_expect_ = track_dis_data.data_;
        // tracking_height_expect_ = track_h_data.data_;

        Eigen::Vector3d goal = goal_data.odom_p_;

        goal_data.odom_p_ = goal;
        goal_data.odom_v_ = Eigen::Vector3d(0.0,0.0,0.0);

        // 每次收到一条新的 goal 消息，都强制视为一次新规划任务
        if(goal_method_==RVIZ_GOAL || goal_method_==FULL_STATE_GOAL)
          get_new_goal = (!has_last_goal_ || goal_data.odom_time_stamp_ms_ != last_goal_stamp_);

        // replan if get new goal
        if (get_new_goal) {
          INFO_MSG("[plan node] get new goal: " << goal_data.odom_p_.transpose());
          plan_res = tlplannerPtr_->plan_goal(init_state, goal_data, traj_now);
          if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
            has_last_goal_ = true;
            last_goal_stamp_ = goal_data.odom_time_stamp_ms_;
            last_goal_ = goal_data.odom_p_;
          }
          INFO_MSG_YELLOW("traj_cur duration: " << traj_now.getTotalDuration());
        }
        // else replan if last traj get occ
        else{
          // bool valid = false;
          // if (has_traj_last_){
          //   valid = tlplannerPtr_->valid_cheack(traj_last_);
          // }
          // if (!valid || emergency_stop_){ 
          if (!traj_last_valid || emergency_stop_){
            std::cout << "emergency_stop_:" << emergency_stop_ << std::endl;
            std::cout << "valid:" << traj_last_valid << std::endl;
            ROS_ERROR("last traj not valid");
            goal_data = last_goal_data_;
            plan_res = tlplannerPtr_->plan_goal(init_state, goal_data, traj_now);
            if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
              has_last_goal_ = true;
              last_goal_stamp_ = goal_data.odom_time_stamp_ms_;
              last_goal_ = goal_data.odom_p_;
            }
            INFO_MSG_YELLOW("traj_cur duration: " << traj_now.getTotalDuration());
          }else{
            // 没有新目标，也没有occ，直接执行上一条轨迹
            no_need_plan = true;
          }
        }
        break;
      }
    default:
      break;
    }

    bool plan_time_valid = true;
    double plan_duration = durationSecond(TimeNow(), t00);
    if (plan_duration > plan_estimated_duration_){
      INFO_MSG_RED("[plan node] Plan Time Exceed: " << plan_duration);
      plan_time_valid = false;
      INFO_MSG_RED("[plan node] Plan Time Exceed Cause Replan Fail.");
    }
    INFO_MSG("[plan node] plan done");


    //! 6. check
    // if no need to replan, directly return
    if (no_need_plan){
      INFO_MSG_GREEN("[plan node] NO NEED TO REPLAN.");
      set_state(NOTNEEDPLAN);
    }
    // if get the new plan result, check it and publish it
    else{
      bool valid = false;
      if (plan_res == tlplanner::TLPlanner::PLANSUCC){
        valid = tlplannerPtr_->valid_cheack(traj_now);
        // if (!plan_time_valid){
        //   valid = false;
        // }
      }
      if (valid){
        traj_now.start_time_ = TimeNow();
        // traj_now.start_time_ = replan_stamp;
        dataManagerPtr_->write_traj(traj_now, dataManagerPtr_->traj_info_);
        traj_last_ = traj_now;
        has_traj_last_ = true;
        INFO_MSG_GREEN("[plan node] REPLAN SUCCESS.");
        emergency_stop_ = false;
        set_state(PLANSUCC);
      }else if (has_traj_last_ && !tlplannerPtr_->valid_cheack(traj_last_)){
        dataManagerPtr_->save_hover_p(odom_p, TimeNow(), dataManagerPtr_->traj_info_);
        INFO_MSG_RED("[plan node] EMERGENCY STOP!");
        emergency_stop_ = true;
        has_traj_last_ = false;
        set_state(PLANFAIL);
        publish_planner_failure("emergency_stop");
      }else{
        INFO_MSG_RED("[plan node] REPLAN FAIL, EXECUTE LAST TRAJ!");
        set_state(PLANFAIL);
        publish_planner_failure("replan_fail");
      }
    }

    TimePoint t1 = TimeNow();
    double d0 = durationSecond(t1, t0) * 1e3; // ms
    if (d0 > thread_dur){
      INFO_MSG_RED("[plan node] Error! thread time exceed! " << d0 << "ms");
      continue;
    }else{
      int tr = floor(thread_dur - d0);
      std::this_thread::sleep_for(std::chrono::milliseconds(tr));
    }
  }
  INFO_MSG_RED("[plan node] Thread Exit.");
   
}
