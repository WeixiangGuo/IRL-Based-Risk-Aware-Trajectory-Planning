#ifndef TLPLANNER
#define TLPLANNER

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"

#include "rotation_util/rotation_util.hpp"
#include "map_interface/map_ros_interface.hpp"
#include "traj_opt/traj_opt.h"
#include "parameter_server/parameter_server.hpp"
#include "prediction/prediction_car.hpp"
#include "plan_env_lod/sdf_map.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/PoseStamped.h>

// #include "rrt_star/rrt_star.h"
// #include "rrt_star/kdtree.h"
// #include "rrt_star/brrt_star.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <env/env.hpp>

#include <thread>
#include <csignal>
#include <cmath>
#include <ctime>
#include <memory>

#define A_STAR
// #define RRT_STAR
// #define B_RRT_STAR

namespace tlplanner{

class TLPlanner{
public:
    enum PlanResState{
        PLANSUCC,
        FAIL,
    };
    std::shared_ptr<traj_opt::TrajOpt> trajoptPtr_;
private:
    std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
    std::shared_ptr<env::Env> envPtr_;
    std::shared_ptr<parameter_server::ParaeterSerer> paraPtr_;
    std::shared_ptr<vis_interface::VisInterface> visPtr_;
    std::shared_ptr<ShareDataManager> dataManagerPtr_;

    std::shared_ptr<clutter_hand::CH_RC_SDF> rc_sdf_ptr_;

    // #ifdef B_RRT_STAR
    // std::shared_ptr<path_plan::BRRTStar> rrt_star_ptr_;
    // #elif defined(RRT_STAR)
    // std::shared_ptr<path_plan::RRTStar> rrt_star_ptr_;
    // #endif

    // tracking plan duration
    double tracking_dur_;
    // sample dt on target trajectory
    double tracking_dt_;
    // tracking expected height, distance and relative angle
    double tracking_height_expect_, tracking_dis_expect_, tracking_angle_expect_;
    
    // landing pos offset
    Eigen::Vector3d land_dp_;
    double land_roll_, land_pitch_;
    
    // estimated time comsume of plan one time
    double plan_estimated_duration_;

    int web_vis_hz_;
    bool clear_traj_robot_markers_before_publish_ = false;
    
    bool is_use_viewpoint_ = false;

    bool is_with_perception_;

    double plan_horizon_len_;

    double v_max_, v_max_grasp_;

    // through-gap ring pose
    geometry_msgs::PoseStamped ring_pose_;
    bool has_ring_pose_ = false;
    bool en_through_ring_ = false;

    // Optional guide path used to shape the MINCO reference route.
    bool s_guide_path_enable_ = false;
    bool s_guide_bypass_astar_ = false;
    double s_guide_endpoint_tolerance_ = 0.45;
    std::vector<Eigen::Vector3d> s_guide_path_;

    std::vector<Eigen::Vector3d> build_s_guide_path(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& goal) const;

public:
    TLPlanner(std::shared_ptr<parameter_server::ParaeterSerer>& para_ptr);
    ~TLPlanner(){}
    inline void set_gridmap_ptr(std::shared_ptr<map_interface::MapInterface>& gridmap_ptr){
        gridmapPtr_ = gridmap_ptr;
    }
    inline void set_env_ptr(std::shared_ptr<env::Env>& env_ptr){
        envPtr_ = env_ptr;
    }
    inline void set_trajopt_ptr(std::shared_ptr<traj_opt::TrajOpt>& trajopt_ptr){
        trajoptPtr_ = trajopt_ptr;
    }
    inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface>& vis_ptr){
        visPtr_ = vis_ptr;
        
        // #if defined(RRT_STAR) or defined(B_RRT_STAR)
        // rrt_star_ptr_->setVisualizer(vis_ptr);
        // #endif
    }
    // inline void set_sdf_ptr(std::shared_ptr<airgrasp::SDFMap>& sdf_map){
    //     sdf_map_ = sdf_map;
    // }

    inline void set_data_ptr(std::shared_ptr<ShareDataManager>& dataManagerPtr){
        dataManagerPtr_ = dataManagerPtr;
    }

    inline void set_rc_sdf_ptr(std::shared_ptr<clutter_hand::CH_RC_SDF>& rc_sdf_ptr){
        rc_sdf_ptr_ = rc_sdf_ptr;
    }

    inline void set_ring_pose(const geometry_msgs::PoseStamped& msg){
        ring_pose_ = msg;
        has_ring_pose_ = true;
    }

    inline void set_en_through_ring(bool en){
        en_through_ring_ = en;
    }

    PlanResState plan_goal(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data);

    bool valid_cheack(const TrajData& traj_data, const TimePoint& cur_t = TimeNow());

    void cal_local_goal_from_path(std::vector<Eigen::Vector3d>& path);


};
} // namespace tlplanner

#endif
