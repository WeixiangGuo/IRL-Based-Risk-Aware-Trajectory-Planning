#ifndef PLANNODE
#define PLANNODE

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"

#include "planning/tlplanner.h"

#include <thread>
#include <random>
#include "ig_planner/data_structures.h"
#include "quadrotor_msgs/ManiGoal.h"
#include "quadrotor_msgs/UAMFullState.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

enum GoalMethod{
    RVIZ_GOAL,
    FULL_STATE_GOAL,
    MANNUAL_SEQUENCE,
    PRE_SEQUENCE,
};

using rot_util = rotation_util::RotUtil;

// #define RVIZ_GOAL

struct PoseInfo
{
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
  Eigen::Vector3d ea;
};

struct Grasp_Pose_Seq_Info
{
  int seq_len;
  int cur_pose_id = 0;
  double grasp_return_height;
  double grasp_entry_dis;
  double grasp_trans_dur;
  double drop_pose_x;
  double drop_pose_y;
  std::vector<PoseInfo> grasp_poses;
};

class Planner{
 public:
  enum PlanMode{
      IDLE,
      GOAL,
      HOVER,
      TRACK,
      LAND,
  };

  enum PlannerState{
    NOTNEEDPLAN,
    PLANFAIL,
    PLANSUCC,
  };

  enum GraspState{
      TO_INIT_POINT,
      INIT_GRASP,
      TO_PRE_GRASP,
      TO_FINAL_GRASP,
      GRASPING,
      TO_PRE_DROP,
      TO_FINAL_DROP,
      DROPPING,
      FINISHED,
  };

  std::shared_ptr<ShareDataManager> dataManagerPtr_;

 private:

  // plan frequency
  int plan_hz_;
  // estimated time comsume of plan one time
  double plan_estimated_duration_;
  // last goal stamp
  TimePoint last_goal_stamp_;
  bool has_last_goal_ = false;
  Eigen::Vector3d last_goal_;
  double last_goal_yaw_;
  // when traj duration less than this threshold, stop plan, execute the last traj
  double land_no_plan_time_;
  // tracking expected height, distance and relative angle
  double tracking_height_expect_, tracking_dis_expect_, tracking_angle_expect_;
  // traj last locally
  bool has_traj_last_ = false;
  TrajData traj_last_;
  // local goal horizon
  double local_horizon_;
  
  GoalMethod goal_method_ = RVIZ_GOAL;

  int web_vis_hz_;

 private:
  // plan mode & state
  PlanMode plan_mode_ = IDLE;
  PlannerState plan_state_;
  std::mutex mode_mutex_, state_mutex_;

  // outer ptr
  // std::shared_ptr<ShareDataManager> dataManagerPtr_;
  std::shared_ptr<parameter_server::ParaeterSerer> paraPtr_;
  std::shared_ptr<vis_interface::VisInterface> visPtr_;
  std::shared_ptr<clutter_hand::CH_RC_SDF> rc_sdf_ptr_;
  
  // inner ptr
  std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajoptPtr_;
  std::shared_ptr<tlplanner::TLPlanner>  tlplannerPtr_;

  Odom last_goal_data_;
  bool emergency_stop_ = false;

  ros::NodeHandle nh_;
  ros::Publisher planner_failure_pub_;

  ros::Subscriber mani_goal_sub_;
  quadrotor_msgs::ManiGoal mani_goal_;
  bool has_mani_goal_ = false;

  int state_cnt_ = 0;
  bool stable_flag_ = false;
  double v_max_, v_max_grasp_;

  // airgrasp
  ros::Subscriber theta_cur_sub_;
  ros::Subscriber irl_safety_distances_sub_;
  bool has_theta_cur_ = false;
  Eigen::Vector3d theta_cur_, dtheta_cur_;

  // ring pose (from pcd_to_pcl)
  ros::Subscriber ring_pose_sub_;
  geometry_msgs::PoseStamped ring_pose_;
  bool has_ring_pose_ = false;
  bool en_through_ring_ = false;

  // random odom sampling
  bool use_random_odom_ = false;
  Eigen::Vector3d odom_rand_pos_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d odom_rand_pos_max_ = Eigen::Vector3d::Zero();
  double odom_rand_yaw_min_ = 0.0, odom_rand_yaw_max_ = 0.0;
  Eigen::Vector3d odom_rand_theta_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d odom_rand_theta_max_ = Eigen::Vector3d::Zero();
  std::mt19937 rng_{std::random_device{}()};
  uint32_t planner_failure_seq_ = 0;

 public:
  Planner(std::shared_ptr<ShareDataManager> dataManagerPtr,
          ros::NodeHandle& nh,
          std::shared_ptr<parameter_server::ParaeterSerer> paraPtr,
          std::shared_ptr<vis_interface::VisInterface> visPtr);

  void plan_thread();

  inline void set_mode(const PlanMode& mode){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    plan_mode_ = mode;
  }

  inline void set_state(const PlannerState& state){
    std::unique_lock<std::mutex> lck(state_mutex_);
    plan_state_ = state;
  }

  inline PlanMode get_mode(){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    return plan_mode_;
  }

  inline std::string get_mode_name(){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    std::string name;
    switch (plan_mode_)
    {
    case IDLE:
      name = "IDLE";
      break;
    case GOAL:
      name = "GOAL";
      break;
    case HOVER:
      name = "HOVER";
      break;
    default:
      name = "Unknow";
    }
    return name;
  }

  inline PlannerState get_state(){
    std::unique_lock<std::mutex> lck(state_mutex_);
    return plan_state_;
  }

  void publish_planner_failure(const std::string& reason);

//! info_map
private:

  void sample_random_odom(Odom& odom_data);
  void irl_safety_distances_cb(const geometry_msgs::Vector3::ConstPtr& msg);

  // bool isInsideBoundaries(const Eigen::Vector4d& point){
  //   Eigen::Vector3d pos = point.head(3);
  //   return sdf_map_->isInMap(pos);
  // }

  void get_line_from_triangle(const Eigen::Matrix3d& triangle, const int& index, 
          Eigen::Vector3d& line_start, Eigen::Vector3d& line_end)
  {
    if(index == 2){
      line_start = triangle.col(2);
      line_end = triangle.col(0);
    }
    else{
      line_start = triangle.col(index);
      line_end = triangle.col((index+1));
    }
  }

  bool intersection_triangles(const Eigen::Matrix3d& triangle0,const Eigen::Matrix3d& triangle1)
  {
    for(int i=0;i<3;i++)
     for(int j=0;j<3;j++)
     {
      Eigen::Vector3d a,b,c,d;
      get_line_from_triangle(triangle0,i,a,b);
      get_line_from_triangle(triangle1,j,c,d);
      if(intersection_lines(a,b,c,d)) return true;
     }
     return false;
  }

  bool intersection_lines(const Eigen::Vector3d& a,const Eigen::Vector3d& b,const Eigen::Vector3d& c,const Eigen::Vector3d& d)
  {
      //快速排斥实验
      if(max(c.x(),d.x())<min(a.x(),b.x())
      ||max(a.x(),b.x())<min(c.x(),d.x())
      ||max(c.y(),d.y())<min(a.y(),b.y())
      ||max(a.y(),b.y())<min(c.y(),d.y())){
          return false;
      }

      //跨立实验
      if((a-d).cross(c-d).transpose().dot((b-d).cross(c-d)) > 0 && (d-b).cross(a-b).transpose().dot((c-b).cross(a-b)) > 0){
          return  false;
      }
      return true;
  }

  void mani_goal_cb(const quadrotor_msgs::ManiGoal::ConstPtr& msg)
  {
    mani_goal_ = *msg;
    has_mani_goal_ = true;
  }

  void theta_cur_cb(const sensor_msgs::JointState::ConstPtr& msg)
  {
    theta_cur_ << msg->position[0], msg->position[1], msg->position[2];
    dtheta_cur_ << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    has_theta_cur_ = true;
  }

  void ring_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    ring_pose_ = *msg;
    has_ring_pose_ = true;
  }
                
private:
  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;

  // Keep track of the best node and its score
  // RRTNode* best_node_;
  // RRTNode* best_branch_root_;

  // kd tree for finding nearest neighbours
  // kdtree* kd_tree_;

  int eval_idx_ = 0; // 防止和best_brance重复更新

  Grasp_Pose_Seq_Info grasp_pose_seq_;
  std::vector<Eigen::Vector3d> guide_pt_vec_;
  int guide_pt_num_ = 0, cur_guide_pt_idx_ = -1;
  GraspState grasp_state_ = TO_INIT_POINT, last_grasp_state_ = TO_INIT_POINT;
  PoseInfo last_end_pose_goal_, end_pose_goal_;
  ros::Time grasp_trans_stamp_;
};



#endif
