#pragma once
#include "util_gym/util_gym.hpp"
#include <mutex>
#include <plan_env_lod/sdf_map.h>
#include <quadrotor_msgs/SyncFrame.h>
namespace map_interface
{
class MapInterface{
 private:
  std::shared_ptr<airgrasp::SDFMap> tgt_map_ptr_;
  std::shared_ptr<airgrasp::SDFMap> env_map_ptr_;
  ros::NodeHandle nh_;
  ros::Subscriber sync_sub_;
  Eigen::Vector3d cam_pos_last_;
  Eigen::Quaterniond cam_quat_last_;
  ros::Time last_time_;
  bool use_world_pcl_ = false;
  bool clear_tgt_from_env_map_ = true;
  bool merge_tgt_into_env_map_ = false;
  bool irl_groups_from_world_pcl_ = false;
  bool irl_split_env_pcl_by_x_ = false;
  double irl_split_x_ = 0.0;
  mutable std::mutex irl_world_pts_mutex_;
  std::vector<Eigen::Vector3d> irl_env_world_pts_;
  std::vector<Eigen::Vector3d> irl_tgt_world_pts_;
  
  public:
    MapInterface(ros::NodeHandle &nh)
    {
        ROS_INFO("[MapInterface] initializing");
        // 1️⃣ 初始化 tgt_map
        tgt_map_ptr_ = std::make_shared<airgrasp::SDFMap>();
        initSingleMap(nh, "tgt_map", tgt_map_ptr_);
        ROS_INFO("[MapInterface] tgt_map initialized");

        // 2️⃣ 初始化 env_map
        env_map_ptr_ = std::make_shared<airgrasp::SDFMap>();
        initSingleMap(nh, "env_map", env_map_ptr_);
        ROS_INFO("[MapInterface] env_map initialized");

        nh.param("use_world_pcl", use_world_pcl_, false);
        nh.param("clear_tgt_from_env_map", clear_tgt_from_env_map_, true);
        nh.param("merge_tgt_into_env_map", merge_tgt_into_env_map_, false);
        nh.param("irl_groups_from_world_pcl", irl_groups_from_world_pcl_, false);
        nh.param("irl_split_env_pcl_by_x", irl_split_env_pcl_by_x_, false);
        nh.param("irl_split_x", irl_split_x_, 0.0);

        // 3️⃣ 订阅 SyncFrame 话题
        std::string syncframe_topic;
        nh.param("syncframe_topic", syncframe_topic, std::string("/sync_frame_pcl"));

        if(use_world_pcl_)
          sync_sub_ = nh.subscribe(syncframe_topic, 10, &MapInterface::syncFrameWorldCallback, this);
        else 
          sync_sub_ = nh.subscribe(syncframe_topic, 10, &MapInterface::syncFrameCallback, this);
    }


  private:

    void initSingleMap(ros::NodeHandle &nh, const std::string &prefix, std::shared_ptr<airgrasp::SDFMap> map){
        std::cout << "Initializing " << prefix << std::endl;  
        map->initMap(nh);
        std::cout << "Setting parameters for " << prefix << std::endl;

        double res;
        Eigen::Vector3d map_size, map_center;
        nh.param(prefix + "/resolution", res, -1.0);
        nh.param(prefix + "/map_size_x", map_size(0), -1.0);
        nh.param(prefix + "/map_size_y", map_size(1), -1.0);
        nh.param(prefix + "/map_size_z", map_size(2), -1.0);
        nh.param(prefix + "/map_center_x", map_center(0), -1.0);
        nh.param(prefix + "/map_center_y", map_center(1), -1.0);
        nh.param(prefix + "/map_center_z", map_center(2), -1.0);

        std::cout << prefix << " res: " << res << std::endl;
        std::cout << prefix << " map_size: " << map_size.transpose() << std::endl;
        std::cout << prefix << " map_center: " << map_center.transpose() << std::endl;
        
        map->setMapParam(res, map_size, map_center, prefix);
    }

    void syncFrameCallback(const quadrotor_msgs::SyncFrame::ConstPtr& msg){

        // 取出 body_odom
        const nav_msgs::Odometry& body_odom = msg->body_odom;
        Eigen::Vector3d position;
        position << msg->body_odom.pose.pose.position.x,
                    msg->body_odom.pose.pose.position.y,
                    msg->body_odom.pose.pose.position.z;

        Eigen::Quaterniond cam_quat;
        cam_quat.x() = msg->body_odom.pose.pose.orientation.x;
        cam_quat.y() = msg->body_odom.pose.pose.orientation.y;
        cam_quat.z() = msg->body_odom.pose.pose.orientation.z;
        cam_quat.w() = msg->body_odom.pose.pose.orientation.w;

        // double err = quatAngularDistanceRad(cam_quat, cam_quat_last_);
        // if(err < 30.0/180.0*M_PI && (position - cam_pos_last_).norm() < 0.05 && (msg->header.stamp - last_time_).toSec() < 0.1){
        //     ROS_WARN_STREAM("ros & pos & timeStamp change too small, skip this frame. err: " << err*180.0/M_PI << " deg");
        //     return;
        // }
        // cam_quat_last_ = cam_quat;
        // cam_pos_last_ = position;
        // last_time_ = msg->header.stamp;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        pcl::fromROSMsg(msg->tgt_pcl, cloud);
        tgt_map_ptr_->inputPointCloud(cloud, cloud.points.size(), position);
        tgt_map_ptr_->vis();

        pcl::fromROSMsg(msg->env_pcl, cloud);
        env_map_ptr_->inputPointCloud(cloud, cloud.points.size(), position);
        env_map_ptr_->vis();
    }

    void syncFrameWorldCallback(const quadrotor_msgs::SyncFrame::ConstPtr& msg){
        pcl::PointCloud<pcl::PointXYZ> cloud_tgt, cloud_env;
        
        pcl::fromROSMsg(msg->tgt_pcl, cloud_tgt);
        pcl::fromROSMsg(msg->env_pcl, cloud_env);

        if (irl_groups_from_world_pcl_) {
          std::lock_guard<std::mutex> lock(irl_world_pts_mutex_);
          irl_env_world_pts_.clear();
          irl_tgt_world_pts_.clear();
          if (irl_split_env_pcl_by_x_) {
            irl_env_world_pts_.reserve(cloud_env.points.size());
            irl_tgt_world_pts_.reserve(cloud_env.points.size() + cloud_tgt.points.size());
            for (const auto &pt : cloud_env.points) {
              if (pt.x < irl_split_x_) {
                irl_env_world_pts_.emplace_back(pt.x, pt.y, pt.z);
              } else {
                irl_tgt_world_pts_.emplace_back(pt.x, pt.y, pt.z);
              }
            }
            for (const auto &pt : cloud_tgt.points) {
              irl_tgt_world_pts_.emplace_back(pt.x, pt.y, pt.z);
            }
            ROS_INFO_THROTTLE(2.0,
                              "[MapInterface][IRL] split env_pcl by x=%.3f into group1/group2: %zu / %zu pts",
                              irl_split_x_, irl_env_world_pts_.size(),
                              irl_tgt_world_pts_.size());
          } else {
            irl_env_world_pts_.reserve(cloud_env.points.size());
            irl_tgt_world_pts_.reserve(cloud_tgt.points.size());
            for (const auto &pt : cloud_env.points) {
              irl_env_world_pts_.emplace_back(pt.x, pt.y, pt.z);
            }
            for (const auto &pt : cloud_tgt.points) {
              irl_tgt_world_pts_.emplace_back(pt.x, pt.y, pt.z);
            }
          }
        }

        tgt_map_ptr_->inputPointCloudWorld(cloud_tgt, true);
        tgt_map_ptr_->vis();

        pcl::PointCloud<pcl::PointXYZ> cloud_env_for_planning = cloud_env;
        if (merge_tgt_into_env_map_) {
          cloud_env_for_planning += cloud_tgt;
        }
        env_map_ptr_->inputPointCloudWorld(cloud_env_for_planning);
        if (clear_tgt_from_env_map_) {
          env_map_ptr_->setPointCloudFree(cloud_tgt);
        }
        env_map_ptr_->vis();
    }

    double quatAngularDistanceRad(const Eigen::Quaterniond& q1,
                                        const Eigen::Quaterniond& q2)
    {
        // 归一化，避免非单位带来的误差
        Eigen::Quaterniond a = q1.normalized();
        Eigen::Quaterniond b = q2.normalized();

        // 四元数点积（注意 q 与 -q 等价，取绝对值得到最小角）
        double d = std::abs(a.coeffs().dot(b.coeffs()));   // coeffs() 顺序为 (x,y,z,w)

        // 数值钳制，防止 acos(>1 or < -1)
        d = std::min(1.0, std::max(-1.0, d));

        // 角度（弧度）：2 * arccos(dot)
        return 2.0 * std::acos(d);
    }

    void filterCachedAABBPoints(const std::vector<Eigen::Vector3d>& src,
                                std::vector<Eigen::Vector3d>& dst,
                                const Eigen::Vector3d& start_pos,
                                const Eigen::Vector3d& end_pos,
                                const Eigen::Vector3d& box_size) const {
      Eigen::Vector3d min_cut_p, max_cut_p;
      for (int i = 0; i < 3; ++i) {
        min_cut_p(i) = std::min(start_pos(i), end_pos(i)) - 0.5 * box_size(i);
        max_cut_p(i) = std::max(start_pos(i), end_pos(i)) + 0.5 * box_size(i);
      }
      dst.clear();
      for (const auto& pt : src) {
        if ((pt.array() >= min_cut_p.array()).all() &&
            (pt.array() <= max_cut_p.array()).all()) {
          dst.push_back(pt);
        }
      }
    }

  public:
    // ------ partical ------
    inline const bool isOccupied(const Eigen::Vector3d& p) const {
      return !env_map_ptr_->isValid(p);
    }

    inline const bool isOccupied(const Eigen::Vector3i& id) const {
      return !env_map_ptr_->isValid(id);
    }

    inline bool checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) const {
      return env_map_ptr_->checkRayValid(p0, p1);
    }
    // ------ partical ------


    // ------ whole-body ------
    // inline const bool isOccupied(const Eigen::VectorXd& state) const {
    //     return env_map_ptr_->isOccupied(state);
    // }

    inline bool checkRayValid(const Eigen::VectorXd& s0, const Eigen::VectorXd& s1) const {
      return env_map_ptr_->checkRayValid(s0,  s1);
    }
    // ------ whole-body ------

    inline double getCostWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) const{
      return env_map_ptr_->getDistWithGrad(pos, grad);
    }
    inline const Eigen::Vector3i pos2idx(const Eigen::Vector3d& pt) const {
      Eigen::Vector3i id;
      env_map_ptr_->posToIndex(pt,id);
      return id;
    }
    inline const Eigen::Vector3d idx2pos(const Eigen::Vector3i& id) const {
      Eigen::Vector3d pt;
      env_map_ptr_->indexToPos(id,pt);
      return pt;
    }
    inline double resolution() const{
      return env_map_ptr_->getResolution();
    }

    inline void getAABBPoints(Eigen::MatrixXd& aabb_pts, 
                              const Eigen::Vector3d& start_pos, 
                              const Eigen::Vector3d& end_pos,
                              const Eigen::Vector3d& box_size) const{
      env_map_ptr_->getAABBPoints(aabb_pts, start_pos, end_pos, box_size);
    }

    inline void getAABBPoints(std::vector<Eigen::Vector3d>& aabb_pts,
                              const Eigen::Vector3d& start_pos, 
                              const Eigen::Vector3d& end_pos,
                              const Eigen::Vector3d& box_size) const{
        std::vector<Eigen::Vector3d> aabb_pts_env, aabb_pts_tgt;

        // 分别从环境地图和目标地图取样
        env_map_ptr_->getAABBPoints(aabb_pts_env, start_pos, end_pos, box_size);
        tgt_map_ptr_->getAABBPoints(aabb_pts_tgt, start_pos, end_pos, box_size);

        aabb_pts_env = env_map_ptr_->farthestPointSampling(aabb_pts_env, 2000);
        aabb_pts_tgt = tgt_map_ptr_->farthestPointSampling(aabb_pts_tgt, 50);

        aabb_pts.clear();
        aabb_pts.reserve(aabb_pts_env.size() + aabb_pts_tgt.size());
        aabb_pts.insert(aabb_pts.end(), aabb_pts_env.begin(), aabb_pts_env.end());
        aabb_pts.insert(aabb_pts.end(), aabb_pts_tgt.begin(), aabb_pts_tgt.end());

        if (aabb_pts.size() == 0) {
            std::cout << "Warning: getAABBPoints returned 0 points!" << std::endl;
            return;
        }

        std::cout << "getAABBPoints get " << aabb_pts.size() << " pts" << std::endl;
        std::cout << "aabb_pts: " << aabb_pts[0].transpose() << std::endl;

        // aabb_pts.resize(aabb_pts_env.size() + aabb_pts_tgt.size());
        // std::copy(aabb_pts_env.begin(), aabb_pts_env.end(), aabb_pts.begin());
        // std::copy(aabb_pts_tgt.begin(), aabb_pts_tgt.end(), aabb_pts.begin() + aabb_pts_env.size());

    }

    inline void getAABBPoints(std::vector<Eigen::Vector3d>& aabb_pts_env,
                              std::vector<Eigen::Vector3d>& aabb_pts_tgt,
                              const Eigen::Vector3d& start_pos,
                              const Eigen::Vector3d& end_pos,
                              const Eigen::Vector3d& box_size) const{
        if (irl_groups_from_world_pcl_) {
          std::vector<Eigen::Vector3d> env_cached, tgt_cached;
          {
            std::lock_guard<std::mutex> lock(irl_world_pts_mutex_);
            env_cached = irl_env_world_pts_;
            tgt_cached = irl_tgt_world_pts_;
          }
          filterCachedAABBPoints(env_cached, aabb_pts_env, start_pos, end_pos, box_size);
          filterCachedAABBPoints(tgt_cached, aabb_pts_tgt, start_pos, end_pos, box_size);
          aabb_pts_env = env_map_ptr_->farthestPointSampling(aabb_pts_env, 2000);
          aabb_pts_tgt = tgt_map_ptr_->farthestPointSampling(aabb_pts_tgt, 2000);
          std::cout << "getAABBPoints cached group1/group2: "
                    << aabb_pts_env.size() << " / " << aabb_pts_tgt.size()
                    << " pts" << std::endl;
          return;
        }

        env_map_ptr_->getAABBPoints(aabb_pts_env, start_pos, end_pos, box_size);
        tgt_map_ptr_->getAABBPoints(aabb_pts_tgt, start_pos, end_pos, box_size);

        aabb_pts_env = env_map_ptr_->farthestPointSampling(aabb_pts_env, 2000);
        aabb_pts_tgt = tgt_map_ptr_->farthestPointSampling(aabb_pts_tgt, 50);

        std::cout << "getAABBPoints split env/tgt: "
                  << aabb_pts_env.size() << " / " << aabb_pts_tgt.size()
                  << " pts" << std::endl;
    }

    inline void getAABBPointsSample(Eigen::MatrixXd& aabb_pts, 
                              const std::vector<Eigen::Vector3d>& sample_pts,
                              const Eigen::Vector3d& box_size) const{
      // env_map_ptr_->getAABBPoints(aabb_pts, sample_pts);
    }

    inline void getAABBPointsSample(std::vector<Eigen::Vector3d>& aabb_pts,
                                    const std::vector<Eigen::Vector3d>& sample_pts,
                                    const Eigen::Vector3d& box_size) const {
        std::vector<Eigen::Vector3d> aabb_pts_env, aabb_pts_tgt;

        // 分别从环境地图和目标地图取样
        env_map_ptr_->getAABBPointsSample(aabb_pts_env, sample_pts, box_size);
        tgt_map_ptr_->getAABBPointsSample(aabb_pts_tgt, sample_pts, box_size);

        // 清空输出 vector
        aabb_pts.clear();

        // 合并两部分
        aabb_pts.resize(aabb_pts_env.size() + aabb_pts_tgt.size());
        std::copy(aabb_pts_env.begin(), aabb_pts_env.end(), aabb_pts.begin());
        std::copy(aabb_pts_tgt.begin(), aabb_pts_tgt.end(), aabb_pts.begin() + aabb_pts_env.size());

    }


    inline void getAABBPointsSample(std::vector<Eigen::Vector3d>& aabb_pts_env,
                                    std::vector<Eigen::Vector3d>& aabb_pts_tgt,
                                    const std::vector<Eigen::Vector3d>& sample_pts,
                                    const Eigen::Vector3d& box_size) const{
      env_map_ptr_->getAABBPointsSample(aabb_pts_env, sample_pts, box_size);
      tgt_map_ptr_->getAABBPointsSample(aabb_pts_tgt, sample_pts, box_size);
    }


    inline Eigen::Vector3d getMinBound() const{
      return env_map_ptr_->getMinBound();
    }

    inline Eigen::Vector3d getMaxBound() const{
      return env_map_ptr_->getMaxBound();
    }

};


} // namespace map_interface
