#include "plan_env_lod/sdf_map.h"
#include "plan_env_lod/map_ros.h"
#include <plan_env_lod/raycast.h>

#include <memory>

#define USE_GS
namespace airgrasp {
    SDFMap::SDFMap() = default;

    SDFMap::~SDFMap() = default;

    void SDFMap::initMap(ros::NodeHandle &nh) {
        mp_ = std::make_unique<MapParam>();
        md_ = std::make_unique<MapData>();
        mr_ = std::make_unique<MapROS>();
        nh_ = nh;
    }

    void SDFMap::setMapParam(const double &res,const Eigen::Vector3d &map_size, 
                            const Eigen::Vector3d &map_center, const std::string &map_name){
        // Params of map properties
        double x_size, y_size, z_size;
        mp_->resolution_ = res;
        x_size = map_size(0);
        y_size = map_size(1);
        z_size = map_size(2);
        mp_->map_center_ = map_center;
        map_name_ = map_name;
        
        // nh_.param("sdf_map/resolution", mp_->resolution_, -1.0);
        // nh_.param("sdf_map/map_size_x", x_size, -1.0);
        // nh_.param("sdf_map/map_size_y", y_size, -1.0);
        // nh_.param("sdf_map/map_size_z", z_size, -1.0);
        // nh_.param("sdf_map/map_center_x", mp_->map_center_(0), -1.0);
        // nh_.param("sdf_map/map_center_y", mp_->map_center_(1), -1.0);
        // nh_.param("sdf_map/map_center_z", mp_->map_center_(2), -1.0);

        nh_.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
        nh_.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
        nh_.param("sdf_map/local_map_margin", mp_->local_map_margin_, 1);
        nh_.param("sdf_map/ground_height", mp_->ground_height_, 1.0);
        nh_.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
        nh_.param("sdf_map/optimistic", mp_->optimistic_, true);
        nh_.param("sdf_map/signed_dist", mp_->signed_dist_, false);

        mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
        mp_->resolution_inv_ = 1 / mp_->resolution_;
        // mp_->map_origin_ = mp_->map_center_ + Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_->ground_height_);
        mp_->map_origin_ = mp_->map_center_ + Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, -z_size / 2.0);
        mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
        for (Eigen::Index i = 0; i < 3; ++i)
            mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
        mp_->map_min_boundary_ = mp_->map_origin_;
        mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
        INFO_MSG("Map size: " << mp_->map_size_.transpose());
        INFO_MSG("Map voxel num: " << mp_->map_voxel_num_.transpose());
        INFO_MSG("Map origin: " << mp_->map_origin_.transpose());

        // Params of raycasting-based fusion
        nh_.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
        nh_.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
        nh_.param("sdf_map/p_min", mp_->p_min_, 0.12);
        nh_.param("sdf_map/p_max", mp_->p_max_, 0.97);
        nh_.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
        nh_.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
        nh_.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);

        auto logit = [](const double &x) { return log(x / (1 - x)); };
        mp_->prob_hit_log_ = logit(mp_->p_hit_);
        mp_->prob_miss_log_ = logit(mp_->p_miss_);
        mp_->clamp_min_log_ = logit(mp_->p_min_);
        mp_->clamp_max_log_ = logit(mp_->p_max_);
        mp_->min_occupancy_log_ = logit(mp_->p_occ_);
        mp_->unknown_flag_ = 0.01;
        cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
             << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
             << ", thresh: " << mp_->min_occupancy_log_ << endl;

        // Initialize data buffer of map
        int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
        md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
        md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
        md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
        md_->count_hit_ = vector<short>(buffer_size, 0);
        md_->count_miss_ = vector<short>(buffer_size, 0);
        md_->flag_rayend_ = vector<char>(buffer_size, -1);
        md_->flag_visited_ = vector<char>(buffer_size, -1);
        md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
        md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
        md_->vis_buffer_ = vector<bool>(buffer_size, false);

        // gs map
        md_->p_mean_buffer_ = vector<Eigen::Vector3d>(buffer_size, Eigen::Vector3d(0,0,0));
        md_->p_temp_sum_buffer_ = vector<Eigen::Vector3d>(buffer_size, Eigen::Vector3d(0,0,0));

        md_->raycast_num_ = 0;
        md_->reset_updated_box_ = true;
        md_->update_min_ = md_->update_max_ = Eigen::Vector3d(0, 0, 0);

        // Try retriving bounding box of map, set box to map size if not specified
        vector<string> axis = {"x", "y", "z"};
        for (Eigen::Index i = 0; i < 3; ++i) {
            nh_.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
            nh_.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
        }
        posToIndex(mp_->box_mind_, mp_->box_min_);
        posToIndex(mp_->box_maxd_, mp_->box_max_);

        // Initialize ROS wrapper
        mr_->setMap(this);
        mr_->setMapName(map_name_);
        mr_->node_ = nh_;
        mr_->init();

        caster_ = std::make_unique<RayCaster>();
        caster_->setParams(mp_->resolution_, mp_->map_origin_);

        // Initialize visualization
        vis_ptr_ = std::make_shared<visualization_airgrasp::Visualization>(nh_);
        mr_->setVisPtr(vis_ptr_);
        std::vector<std::vector<Eigen::Vector3d>> path_list;
        std::vector<std::pair<Eigen::Vector3d,double>> pcl_i;
        std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> arrows;
        std::vector<Eigen::Vector3d> pcl;

        cam_param_ = mr_->getCamParam();
        fov_h_ = 2 * atan(cam_param_.cx / cam_param_.fx);
        fov_v_ = 2 * atan(cam_param_.cy / cam_param_.fy);

        //! debug
    }

    void SDFMap::resetBuffer() {
        resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
        md_->local_bound_min_ = Eigen::Vector3i::Zero();
        md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    }

    void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos) {
        Eigen::Vector3i min_id, max_id;
        posToIndex(min_pos, min_id);
        posToIndex(max_pos, max_id);
        boundIndex(min_id);
        boundIndex(max_id);

        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int z = min_id(2); z <= max_id(2); ++z) {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                    md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
                }
    }

    void SDFMap::resetVisBuffer(){

        while (vis_lock_.test_and_set())
            ;

        for (int x = 0; x < mp_->map_voxel_num_(0); ++x)
            for (int y = 0; y < mp_->map_voxel_num_(1); ++y)
                for (int z = 0; z < mp_->map_voxel_num_(2); ++z) {
                    md_->vis_buffer_[toAddress(x, y, z)] = false;
                }

        vis_lock_.clear();
    }


    template<typename F_get_val, typename F_set_val>
    void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
        int v[mp_->map_voxel_num_(dim)];
        double z[mp_->map_voxel_num_(dim) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void SDFMap::updateESDF3d() {
        Eigen::Vector3i min_esdf = md_->local_bound_min_;
        Eigen::Vector3i max_esdf = md_->local_bound_max_;

        if (mp_->optimistic_) {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                    fillESDF(
                            [&](int z) {
                                return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                                       0 :
                                       std::numeric_limits<double>::max();
                            },
                            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                            max_esdf[2], 2);
                }
        } else {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                    fillESDF(
                            [&](int z) {
                                int adr = toAddress(x, y, z);
                                // 计算free的填入inf和unk
                                return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                                        md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ?
                                       0 :
                                       std::numeric_limits<double>::max();
                            },
                            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                            max_esdf[2], 2);
                }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF(
                        [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                        [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                        max_esdf[1], 1);
            }
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                fillESDF(
                        [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                        [&](int x, double val) {
                            md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                        },
                        min_esdf[0], max_esdf[0], 0);
            }


        // set tmp_buffer to 0
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                    md_->tmp_buffer1_[toAddress(x, y, z)] = 0;
                    md_->tmp_buffer2_[toAddress(x, y, z)] = 0;
                    // std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
                    // std::cout << md_->distance_buffer_[toAddress(x, y, z)] << std::endl;

                }

        if (mp_->signed_dist_) {
            // Compute negative distance
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
                    fillESDF(
                            [&](int z) {
                                int adr = toAddress(x, y, z);
                                // 计算unk和inf的填入free
                                return /*md_->occupancy_buffer_inflate_[adr] == 0 ||*/
                                        md_->occupancy_buffer_[adr] >= mp_->clamp_min_log_ - 1e-3 ?
                                       0 :
                                       std::numeric_limits<double>::max();
                            },
                            [&](int z, double val) { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                            max_esdf[2], 2);
                }
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                    fillESDF(
                            [&](int y) { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                            [&](int y, double val) { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                            max_esdf[1], 1);
                }
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
                    fillESDF(
                            [&](int x) { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                            [&](int x, double val) {
                                md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                            },
                            min_esdf[0], max_esdf[0], 0);
                }
            // Merge negative distance with positive
            for (int x = min_esdf(0); x <= max_esdf(0); ++x)
                for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                    for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
                        int idx = toAddress(x, y, z);
                        if (md_->distance_buffer_neg_[idx] > 0.0)
                            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
                    }
        }
    }

    void SDFMap::setCacheOccupancy(const int &adr, const int &occ) {
        // Add to update list if first visited
        if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0) md_->cache_voxel_.push(adr);

        if (occ == 0)
            md_->count_miss_[adr] = 1;
        else if (occ == 1)
            md_->count_hit_[adr] += 1;
    }

    // mask pcl
    // [修改 1] 函数签名增加 setOcc
    void SDFMap::inputPointCloud(
            const pcl::PointCloud<pcl::PointXYZ> &cloud_mask, const int &point_num,
            const Eigen::Vector3d &camera_pos, const bool &setOcc) {
        if (point_num == 0) return;
        md_->raycast_num_ += 1;

        Eigen::Vector3d update_min = camera_pos;
        Eigen::Vector3d update_max = camera_pos;
        if (md_->reset_updated_box_) {
            md_->update_min_ = camera_pos;
            md_->update_max_ = camera_pos;
            md_->reset_updated_box_ = false;
        }

        Eigen::Vector3d pt_w, tmp;
        Eigen::Vector3i idx;
        int vox_adr;
        double length;
        double mask_threshold = 0.5;
        for (size_t i = 0; i < point_num; ++i) {

            // debug
            ros::Time t_loop;

            auto &pt = cloud_mask.points[i];
            pt_w << pt.x, pt.y, pt.z;
            int tmp_flag;

            t_loop = ros::Time::now();
            bool is_cam_in_map, is_pt_in_map;
            Eigen::Vector3d pt_in_map(pt_w), cam_in_map(camera_pos);
            int count = intersectSegmentAABB_fast(camera_pos, pt_w, \
                        is_cam_in_map, is_pt_in_map, cam_in_map, pt_in_map);

            if(count==0) continue;

            pt_w = pt_in_map;
            if(!isInMap(pos2idx(pt_in_map))){
                INFO_MSG_RED("pt_w out of map");
                continue;
            }

            if(!isInMap(pos2idx(cam_in_map))){
                INFO_MSG_RED("cam_in_map out of map");
                continue;
            }

            if (!is_pt_in_map) {
                // pt_w = closetPointInMap(pt_w, cam_in_map);
                length = (pt_in_map - cam_in_map).norm();
                if (length > mp_->max_ray_length_)
                    pt_in_map = (pt_in_map - cam_in_map) / length * mp_->max_ray_length_ + cam_in_map;
                if (pt_in_map[2] < 0.2) continue;
                tmp_flag = 0;
            } else {
                length = (pt_in_map - cam_in_map).norm();
                if (length > mp_->max_ray_length_) {
                    pt_in_map = (pt_in_map - cam_in_map) / length * mp_->max_ray_length_ + cam_in_map;
                    if (pt_in_map[2] < 0.2) continue;
                    tmp_flag = 0;
                } else {
                    // [修改 2] 关键逻辑控制
                    // 如果 setOcc=true，末端标记为 1 (Hit)，会增加占据概率
                    // 如果 setOcc=false，末端标记为 0 (Miss)，连同射线路径一起进行清除操作
                    tmp_flag = setOcc ? 1 : 0;
                }
            }

            posToIndex(pt_in_map, idx);
            vox_adr = toAddress(idx);
            
            // setCacheOccupancy 内部会统计 count_hit_ 或 count_miss_
            // 如果 tmp_flag 是 0，则 count_miss_++，最终更新时概率会下降
            setCacheOccupancy(vox_adr, tmp_flag);

            for (int k = 0; k < 3; ++k) {
                update_min[k] = min(update_min[k], pt_in_map[k]);
                update_max[k] = max(update_max[k], pt_in_map[k]);
            }

            // ! gs map
            // [修改 3] 只有当它是占据点(setOcc=true)时，才更新高斯映射的平滑位置
            // 如果是在清除障碍(setOcc=false)，不应该把清除点的位置累加进去
            if (setOcc) {
                Eigen::Vector3d offset = pt_in_map - idx2pos(idx);
                md_->p_temp_sum_buffer_[vox_adr] += offset;
            }

            // ! Raycasting
            if (md_->flag_rayend_[vox_adr] == md_->raycast_num_) continue;
            md_->flag_rayend_[vox_adr] = md_->raycast_num_;

            
            caster_->input(pt_in_map, cam_in_map);
            caster_->nextId(idx);
            while (caster_->nextId(idx)) {
                if(!isInMap(idx))
                    continue;   
                setCacheOccupancy(toAddress(idx), 0);
            }

        }

        // **更新 bounding box**
        Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
        posToIndex(update_max + bound_inf, md_->local_bound_max_);
        posToIndex(update_min - bound_inf, md_->local_bound_min_);
        boundIndex(md_->local_bound_min_);
        boundIndex(md_->local_bound_max_);
        mr_->local_updated_ = true;

        for (int k = 0; k < 3; ++k) {
            md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
            md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
        }

        // update global buffer
        while (!md_->cache_voxel_.empty()) {
            int adr = md_->cache_voxel_.front();
            md_->cache_voxel_.pop();

            // update occ
            // 这里会自动根据 count_hit_ 和 count_miss_ 的大小关系决定是加 prob_hit 还是 prob_miss
            // 如果 setOcc=false，则 count_hit_ 为 0，count_miss_ > 0，这里会走 prob_miss_log_ 分支
            double log_odds_update = md_->count_hit_[adr] >= md_->count_miss_[adr] 
                                   ? mp_->prob_hit_log_ : mp_->prob_miss_log_;

            // ! update gs map
            if (md_->count_hit_[adr] > 0) {
                double k = std::pow(0.95, md_->count_hit_[adr]);
                md_->p_mean_buffer_[adr] = md_->p_mean_buffer_[adr] * k
                                        + md_->p_temp_sum_buffer_[adr] / md_->count_hit_[adr] * (1 - k);
                md_->p_temp_sum_buffer_[adr] = Eigen::Vector3d(0,0,0);
            }

            // reset count
            md_->count_hit_[adr] = md_->count_miss_[adr] = 0;

            if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;

            md_->occupancy_buffer_[adr] = std::min(
                    std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
                    mp_->clamp_max_log_);
        }
    }

    void SDFMap::inputPointCloudWorld(
            const pcl::PointCloud<pcl::PointXYZ> &cloud, const bool &reset_map_center){
        
        // 0. 如果需要，重新设置地图中心为点云中心
        if (reset_map_center && cloud.points.size() > 0) {
            // 计算点云中心
            Eigen::Vector3d cloud_center(0, 0, 0);
            for (size_t i = 0; i < cloud.points.size(); ++i) {
                cloud_center(0) += cloud.points[i].x;
                cloud_center(1) += cloud.points[i].y;
                cloud_center(2) += cloud.points[i].z;
            }
            cloud_center /= cloud.points.size();
            
            // 更新地图i心和原点
            mp_->map_center_ = cloud_center;
            mp_->map_origin_ = mp_->map_center_ + Eigen::Vector3d(
                -mp_->map_size_(0) / 2.0, 
                -mp_->map_size_(1) / 2.0, 
                -mp_->map_size_(2) / 2.0
            );
            mp_->map_min_boundary_ = mp_->map_origin_;
            mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;
            
            // 更新 raycaster 参数
            // caster_->setParams(mp_->resolution_, mp_->map_origin_);;
            
            ROS_INFO_STREAM("Reset map center to point cloud center: " << cloud_center.transpose());
        }
        
        // 1. 清空所有地图
        int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
        
        // 清空 occupancy buffer
        std::fill(md_->occupancy_buffer_.begin(), md_->occupancy_buffer_.end(), mp_->clamp_min_log_);
        
        // 清空 inflate buffer
        std::fill(md_->occupancy_buffer_inflate_.begin(), md_->occupancy_buffer_inflate_.end(), 0);
        
        // 清空 distance buffer
        // std::fill(md_->distance_buffer_.begin(), md_->distance_buffer_.end(), 0.0);
        // std::fill(md_->distance_buffer_neg_.begin(), md_->distance_buffer_neg_.end(), 0.0);
        
        // 清空 gs map buffers
        std::fill(md_->p_mean_buffer_.begin(), md_->p_mean_buffer_.end(), Eigen::Vector3d(0, 0, 0));
        std::fill(md_->p_temp_sum_buffer_.begin(), md_->p_temp_sum_buffer_.end(), Eigen::Vector3d(0, 0, 0));
        
        // 清空 count buffers
        // std::fill(md_->count_hit_.begin(), md_->count_hit_.end(), 0);
        // std::fill(md_->count_miss_.begin(), md_->count_miss_.end(), 0);
        
        // 清空 cache queue
        // while (!md_->cache_voxel_.empty()) {
        //     md_->cache_voxel_.pop();
        // }

        // 2. 将点云所在 grid 直接设置成 occ，并累积偏移量用于求均值
        int point_num = cloud.points.size();
        
        // 临时计数器，用于统计每个 grid 中有多少个点
        std::vector<int> temp_count(md_->p_temp_sum_buffer_.size(), 0);
        
        for (size_t i = 0; i < point_num; ++i) {
            auto &pt = cloud.points[i];
            Eigen::Vector3d pt_w;
            pt_w << pt.x, pt.y, pt.z;
            
            // 检查点是否在地图内
            if (!isInMap(pt_w)) {
                continue;
            }
            
            // 获取点所在的 grid index
            Eigen::Vector3i idx;
            posToIndex(pt_w, idx);
            int vox_adr = toAddress(idx);
            
            // 直接设置为占据状态 (clamp_max_log_ 表示最大占据概率)
            md_->occupancy_buffer_[vox_adr] = mp_->clamp_max_log_;
            
            // 累积偏移量到临时缓冲区
            Eigen::Vector3d offset = pt_w - idx2pos(idx);
            md_->p_temp_sum_buffer_[vox_adr] += offset;
            temp_count[vox_adr]++;
        }
        
        // 计算每个 grid 的平均偏移量并更新 p_mean_buffer
        for (size_t i = 0; i < point_num; ++i) {
            auto &pt = cloud.points[i];
            Eigen::Vector3d pt_w;
            pt_w << pt.x, pt.y, pt.z;
            
            if (!isInMap(pt_w)) {
                continue;
            }
            
            Eigen::Vector3i idx;
            posToIndex(pt_w, idx);
            int vox_adr = toAddress(idx);
            
            // 只处理一次（第一次遇到的时候）
            if (temp_count[vox_adr] > 0) {
                // 计算平均偏移量
                md_->p_mean_buffer_[vox_adr] = md_->p_temp_sum_buffer_[vox_adr] / temp_count[vox_adr];
                // 清空临时缓冲区
                md_->p_temp_sum_buffer_[vox_adr] = Eigen::Vector3d(0, 0, 0);
                // 标记已处理
                temp_count[vox_adr] = 0;
            }
        }
        
        // 标记地图已更新
        // mr_->local_updated_ = true;
        md_->local_bound_min_ = Eigen::Vector3i::Zero();
        md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
        clearAndInflateLocalMap();
        updateESDF3d();
    }

    void SDFMap::setPointCloudFree(
            const pcl::PointCloud<pcl::PointXYZ> &cloud, const double &inflate_radius) {
        
        if (cloud.points.size() == 0) {
            ROS_WARN("setPointCloudFree: empty point cloud");
            return;
        }
        
        // 计算膨胀步数
        int inflate_step = std::ceil(inflate_radius / mp_->resolution_);
        
        // 遍历点云中的每个点
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            auto &pt = cloud.points[i];
            Eigen::Vector3d pt_w;
            pt_w << pt.x, pt.y, pt.z;
            
            // 检查点是否在地图内
            if (!isInMap(pt_w)) {
                continue;
            }
            
            // 获取点所在的 grid index
            Eigen::Vector3i idx;
            posToIndex(pt_w, idx);
            
            // 如果不需要膨胀，直接设置该点为 free
            if (inflate_step <= 0) {
                int vox_adr = toAddress(idx);
                md_->occupancy_buffer_[vox_adr] = mp_->clamp_min_log_;
                md_->occupancy_buffer_inflate_[vox_adr] = 0;
            } else {
                // 需要膨胀：将该点周围的 grid 也设置为 free
                for (int x = -inflate_step; x <= inflate_step; ++x) {
                    for (int y = -inflate_step; y <= inflate_step; ++y) {
                        for (int z = -inflate_step; z <= inflate_step; ++z) {
                            Eigen::Vector3i idx_inflated = idx + Eigen::Vector3i(x, y, z);
                            
                            // 检查膨胀后的点是否在地图内
                            if (!isInMap(idx_inflated)) {
                                continue;
                            }
                            
                            // 检查是否在球形范围内（可选，更精确）
                            Eigen::Vector3d pt_inflated = idx2pos(idx_inflated);
                            if ((pt_inflated - pt_w).norm() > inflate_radius) {
                                continue;
                            }
                            
                            int vox_adr = toAddress(idx_inflated);
                            md_->occupancy_buffer_[vox_adr] = mp_->clamp_min_log_;
                            md_->occupancy_buffer_inflate_[vox_adr] = 0;
                        }
                    }
                }
            }
        }
        
        ROS_INFO_STREAM("setPointCloudFree: processed " << cloud.points.size() 
                        << " points with inflate_radius=" << inflate_radius);
    }


    Eigen::Vector3d
    SDFMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt) {
        Eigen::Vector3d diff = pt - camera_pt;
        Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
        Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
        double min_t = 1000000;
        for (Eigen::Index i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t) min_t = t1;
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t) min_t = t2;
            }
        }
        return camera_pt + (min_t - 1e-3) * diff;
    }

    int SDFMap::intersectSegmentAABB_fast(
        const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
        bool& p1Inside, bool& p2Inside,
        Eigen::Vector3d& closestP1, Eigen::Vector3d& closestP2)
    {
        const double xmin = mp_->map_min_boundary_.x();
        const double ymin = mp_->map_min_boundary_.y();
        const double zmin = mp_->map_min_boundary_.z();
        const double xmax = mp_->map_max_boundary_.x();
        const double ymax = mp_->map_max_boundary_.y();
        const double zmax = mp_->map_max_boundary_.z();

        const double x1 = p1.x(), y1 = p1.y(), z1 = p1.z();
        const double x2 = p2.x(), y2 = p2.y(), z2 = p2.z();
        const double dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;

        // 判断端点是否在盒内
        p1Inside = (x1 >= xmin && x1 <= xmax &&
                    y1 >= ymin && y1 <= ymax &&
                    z1 >= zmin && z1 <= zmax);
        p2Inside = (x2 >= xmin && x2 <= xmax &&
                    y2 >= ymin && y2 <= ymax &&
                    z2 >= zmin && z2 <= zmax);

        if (p1Inside && p2Inside) {
            closestP1 = clampInsideBox(p1, mp_->map_min_boundary_, mp_->map_max_boundary_, 1e-6);
            closestP2 = clampInsideBox(p2, mp_->map_min_boundary_, mp_->map_max_boundary_, 1e-6);
            return 2;
        }

        double t0 = 0.0, t1 = 1.0;

        // 内联 clip 函数，减少函数调用开销
        auto clip = [&](double p, double q) -> bool {
            if (p == 0.0) return q >= 0.0;
            double r = q / p;
            if (p < 0.0) {
                if (r > t1) return false;
                if (r > t0) t0 = r;
            } else {
                if (r < t0) return false;
                if (r < t1) t1 = r;
            }
            return true;
        };

        // Liang–Barsky 剪裁算法
        if (!clip(-dx, x1 - xmin)) return 0;
        if (!clip( dx, xmax - x1)) return 0;
        if (!clip(-dy, y1 - ymin)) return 0;
        if (!clip( dy, ymax - y1)) return 0;
        if (!clip(-dz, z1 - zmin)) return 0;
        if (!clip( dz, zmax - z1)) return 0;

        // 计算交点（可能为 0,1,2）
        int count = 0;
        const double eps = 1e-12;

        if (t0 >= 0.0 - eps && t0 <= 1.0 + eps) {
            closestP1 = p1 + t0 * (p2 - p1);
            closestP1 = clampInsideBox(closestP1, mp_->map_min_boundary_, mp_->map_max_boundary_);
            count++;
        }
        if (t1 >= 0.0 - eps && t1 <= 1.0 + eps && t1 > t0 + eps) {
            closestP2 = p1 + t1 * (p2 - p1);
            closestP2 = clampInsideBox(closestP2, mp_->map_min_boundary_, mp_->map_max_boundary_);
            count++;
        }

        return count;
    }

    Eigen::Vector3d SDFMap::clampInsideBox(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& box_min,
        const Eigen::Vector3d& box_max,
        double delta)
    {
        Eigen::Vector3d q = p;
        for (int i = 0; i < 3; ++i) {
            // 限制范围：box_min + delta < q[i] < box_max - delta
            if (q[i] <= box_min[i] + delta)
                q[i] = box_min[i] + delta;
            else if (q[i] >= box_max[i] - delta)
                q[i] = box_max[i] - delta;
        }
        return q;
    }

    // 将初末点放在box中
    void SDFMap::intersectSegmentAABB(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                            bool& p1Inside, bool& p2Inside, std::vector<Eigen::Vector3d>& intersections, 
                            Eigen::Vector3d& closestP1, Eigen::Vector3d& closestP2) {

        // Check if endpoints are inside the AABB
        p1Inside = isInMap(p1);
        p2Inside = isInMap(p2);

        if (p1Inside) {
            closestP1 = p1;
            intersections.push_back(p1);
        }
        if (p2Inside) {
            closestP2 = p2;
            intersections.push_back(p2);
        }
        
        if (p1Inside && p2Inside)
            return;

        Eigen::Vector3d dir = p2 - p1;
        Eigen::Vector3d invDir = dir.array().inverse();
        
        Eigen::Vector3d tMin = (mp_->map_min_boundary_ - p1).cwiseProduct(invDir);
        Eigen::Vector3d tMax = (mp_->map_max_boundary_ - p1).cwiseProduct(invDir);
        
        // 进入 aabb 时间
        Eigen::Vector3d t1 = tMin.cwiseMin(tMax);
        // 离开 aabb 时间
        Eigen::Vector3d t2 = tMin.cwiseMax(tMax);
        
        // 最后进入 aabb 的
        double tNear = t1.maxCoeff();
        // 最早离开 aabb 的
        double tFar = t2.minCoeff();
        

        if (tNear < tFar - 2e-4 && tNear < 1.0 - 1e-4 && tFar > 0.0 + 1e-4) {
            if (tNear >= 0.0 && tNear <= 1.0){
                double t_tmp = tNear + 1e-4;
                closestP1 = p1 + t_tmp * dir;
                // INFO_MSG("tNear:"<<tNear);
                // closestP1 = p1 + tNear * dir;
                intersections.push_back(p1 + t_tmp * dir);
            }
            if (tFar >= 0.0 && tFar <= 1.0){
                double t_tmp = tFar - 1e-4;
                closestP2 = p1 + t_tmp * dir;
                // INFO_MSG("tFar:"<<tFar);
                // closestP2 = p1 + tFar * dir;
                intersections.push_back(p1 + t_tmp * dir);
            }
        }
        
        // closestP1 = p1Inside ? p1 : (p1 + tNear * dir + 1e-3 * Eigen::Vector3d::Ones());
        // closestP2 = p2Inside ? p2 : (p2 + tFar * dir - 1e-3 * Eigen::Vector3d::Ones());
        
        // return !intersections.empty();
    }

    void SDFMap::clearAndInflateLocalMap() {
        // update inflated occupied cells
        // clean outdated occupancy
        int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
        vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                }

        // inflate newest occpuied cells
        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z) {
                    int id1 = toAddress(x, y, z);
                    if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_) {
                        inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

                        int buf_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
                        for (const auto &inf_pt: inf_pts) {
                            int idx_inf = toAddress(inf_pt);
                            if (idx_inf >= 0 && idx_inf < buf_size) {
                                md_->occupancy_buffer_inflate_[idx_inf] = 1;
                            }
                        }
                    }
                }

        // add virtual ceiling to limit flight height
        if (mp_->virtual_ceil_height_ > -0.5) {
            int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
            for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
                for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y) {
                    // md_->occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
                    md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
                }
        }
    }

    double SDFMap::getResolution() {
        return mp_->resolution_;
    }

    Eigen::Vector3d SDFMap::getMinBound() {
        return mp_->map_min_boundary_;
    }

    Eigen::Vector3d SDFMap::getMaxBound() {
        return mp_->map_max_boundary_;
    }

    double SDFMap::getInfDis(){
        return ceil(mp_->obstacles_inflation_ / mp_->resolution_)*mp_->resolution_;
    }


    int SDFMap::getVoxelNum() {
        return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
    }

    Eigen::Vector3i SDFMap::get3DVoxelNum(){
        return mp_->map_voxel_num_;
    }


    void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) {
        ori = mp_->map_origin_, size = mp_->map_size_;
    }

    void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax) {
        bmin = mp_->box_mind_;
        bmax = mp_->box_maxd_;
    }

    void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset) {
        bmin = md_->update_min_;
        bmax = md_->update_max_;
        if (reset) md_->reset_updated_box_ = true;
    }

    double SDFMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad) {
        if (!isInMap(pos)) {
            grad.setZero();
            return 0;
        }

        /* trilinear interpolation */
        Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);
        Eigen::Vector3d idx_pos, diff;
        indexToPos(idx, idx_pos);
        diff = (pos - idx_pos) * mp_->resolution_inv_;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++) {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    values[x][y][z] = getDistance(current_idx);
                }

        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        double dist = (1 - diff[2]) * v0 + diff[2] * v1;

        grad[2] = (v1 - v0) * mp_->resolution_inv_;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= mp_->resolution_inv_;

        return dist;
    }

void SDFMap::getRelatedUnkPcl(const std::vector<Eigen::Vector3d> &cams_p, pcl::PointCloud<pcl::PointXYZ>& pcl_unk)
{
    //! 1.get pcl_unknown
    ROS_WARN("get pcl_unknown");
    //! 1.1 get aabb_bound
    //! 1.2 遍历aabb中的点，找到所有的front_unk
//? debug
std::vector<Eigen::Vector3d> pts;

    pcl::PointXYZ pcl;
    Eigen::Vector3d min_cut_p, max_cut_p;

    //TODO fixed me: vis buffer query at the same time
    resetVisBuffer();
    for(const auto& cam_p:cams_p){
        // std::cout << "cam_p: " << cam_p.transpose() << std::endl;
        for(int i=0; i<3; ++i){
            // min_cut_p(i) = cam_p(i)-(cam_param_.max_range+0.3);
            // max_cut_p(i) = cam_p(i)+(cam_param_.max_range+0.3);

        // 作图
            min_cut_p(i) = cam_p(i)-99;
            max_cut_p(i) = cam_p(i)+99;
        }
        
        Eigen::Vector3i min_cut, max_cut;
        posToIndex(min_cut_p, min_cut);
        posToIndex(max_cut_p, max_cut);
        boundIndex(min_cut);
        boundIndex(max_cut);

    //第一层unk
        for(int x=min_cut(0); x<=max_cut(0); x++)
        for(int y=min_cut(1); y<=max_cut(1); y++)
        for(int z=min_cut(2); z<=max_cut(2); z++){
            Eigen::Vector3i id(x, y, z);
            // if(md_->distance_buffer_[toAddress(x,y,z)] + 2*mp_->resolution_ < 1e-3 
            // && md_->distance_buffer_[toAddress(x,y,z)] + 3.3*mp_->resolution_ > 1e-3
            // && !md_->vis_buffer_[toAddress(id)]){
// 作图
            // if(isUnknown(id) && isFrontUnk(id)){

            if(isUnknown(id) && !md_->vis_buffer_[toAddress(id)]){
                
                Eigen::Vector3d pt = idx2pos(id);
                pcl.x = pt(0); pcl.y = pt(1); pcl.z = pt(2);
                pcl_unk.push_back(pcl);
                md_->vis_buffer_[toAddress(id)] = true;
//? debug
pts.push_back(pt);
            }
        }

    }

//? debug
vis_ptr_->visualize_pointcloud(pts, "pcl_front_unk");

}

bool SDFMap::checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1)
{
    if (!isValid(p0) || !isValid(p1))
        return false;

    std::vector<Eigen::Vector3d> ray_pts;
    ray_pts.push_back(p0);
    ray_pts.push_back(p1);

    Eigen::Vector3i idx0, idx1;
    posToIndex(p0, idx0);
    posToIndex(p1, idx1);
    if(idx0 == idx1)
        return true;

    auto caster = std::make_unique<RayCaster>();
    caster->setParams(mp_->resolution_, mp_->map_origin_);

    caster->input(p0, p1);
    Eigen::Vector3i rayIdx = idx0;
    while (caster->nextId(rayIdx))
        if(!isValid(rayIdx))
            return false;
    
    return true;
}

// 如果不set会unknown
void SDFMap::setCamearPose2free(const Eigen::Vector3d& cam_p){
    Eigen::Vector3i id;
    posToIndex(cam_p, id);
    double clear_radius = 0.6;
    int step = clear_radius/mp_->resolution_;
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -0.5*step; z <= 0.5*step; ++z) {
                Eigen::Vector3i idx_temp = Eigen::Vector3i(id(0) + x, id(1) + y, id(2) + z);
                Eigen::Vector3d pos_temp;
                indexToPos(idx_temp, pos_temp);
                if(isInMap(idx_temp) && (pos_temp.head(2)-cam_p.head(2)).norm() < clear_radius)
                {
                    md_->occupancy_buffer_[toAddress(idx_temp)] = mp_->clamp_min_log_;
                    md_->occupancy_buffer_inflate_[toAddress(idx_temp)] = 0;
                }
            }
}

void SDFMap::vis(){
    mr_->publishMapOcc();
    // mr_->publishUnknown();
    // mr_->publishInfo();
    // mr_->publishTarget();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SDFMap::getPointCloudXYZI() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int x = 0; x < mp_->map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_->map_voxel_num_(1); ++y)
    for (int z = 0; z < mp_->map_voxel_num_(2); ++z) {
        Eigen::Vector3d pt;
        indexToPos(Eigen::Vector3i(x, y, z), pt);

        pcl::PointXYZI point;
        point.x = static_cast<float>(pt.x());
        point.y = static_cast<float>(pt.y());
        point.z = static_cast<float>(pt.z());

        if (isUnknown(Eigen::Vector3i(x, y, z))) {
            point.intensity = static_cast<float>(VoxelStatus::UNKNOWN);
        } else if (isOccupied(Eigen::Vector3i(x, y, z))) {
            point.intensity = static_cast<float>(VoxelStatus::OCCUPIED);
        } else if (!isOccupied(Eigen::Vector3i(x, y, z))) {
            point.intensity = static_cast<float>(VoxelStatus::FREE);
        }

        cloud->push_back(point);
    }

    return cloud;
}



void SDFMap::getAABBPoints(Eigen::MatrixXd& aabb_pts, 
                            const Eigen::Vector3d& start_pos, 
                            const Eigen::Vector3d& end_pos,
                            const Eigen::Vector3d& box_size){
    aabb_pts.resize(0,0);
    
    // find local min and max idx
    Eigen::Vector3d min_cut_p, max_cut_p;
    

    for(int i=0; i<3; ++i){
        min_cut_p(i) = std::min(start_pos(i), end_pos(i))-0.5*box_size(i);
        max_cut_p(i) = std::max(start_pos(i), end_pos(i))+0.5*box_size(i);
    }

    Eigen::Vector3i min_cut, max_cut;
    posToIndex(min_cut_p, min_cut);
    posToIndex(max_cut_p, max_cut);
    boundIndex(min_cut);
    boundIndex(max_cut);

    std::vector<Eigen::Vector3d> aabb_pts_vec;
    for(int x=min_cut(0); x<=max_cut(0); x++)
        for(int y=min_cut(1); y<=max_cut(1); y++)
            for(int z=min_cut(2); z<=max_cut(2); z++){
                Eigen::Vector3i id(x, y, z);
                
                if(isOccupied(id))
                    aabb_pts_vec.push_back(idx2pos(id));
            }

    aabb_pts.resize(3,aabb_pts_vec.size());
    for(int i=0; i<aabb_pts_vec.size(); ++i)
        aabb_pts.col(i) = aabb_pts_vec[i];

    std::cout << "aabb_pts: " << aabb_pts.transpose() << std::endl;
}   


void SDFMap::getAABBPoints(std::vector<Eigen::Vector3d>& aabb_pts, 
                            const Eigen::Vector3d& start_pos, 
                            const Eigen::Vector3d& end_pos,
                            const Eigen::Vector3d& box_size){
    // aabb_pts.clear();
    
    // find local min and max idx
    Eigen::Vector3d min_cut_p, max_cut_p;

    for(int i=0; i<3; ++i){
        min_cut_p(i) = std::min(start_pos(i), end_pos(i))-0.5*box_size(i);
        max_cut_p(i) = std::max(start_pos(i), end_pos(i))+0.5*box_size(i);
    }

    Eigen::Vector3i min_cut, max_cut;
    posToIndex(min_cut_p, min_cut);
    posToIndex(max_cut_p, max_cut);
    boundIndex(min_cut);
    boundIndex(max_cut);


    // while (p_mean_lock_.test_and_set())
    //     ;

    for(int x=min_cut(0); x<=max_cut(0); x++)
        for(int y=min_cut(1); y<=max_cut(1); y++)
            for(int z=min_cut(2); z<=max_cut(2); z++){
                Eigen::Vector3i id(x, y, z);
                
                if(isOccupied(id)){
                    // aabb_pts.push_back(idx2pos(id));
                    
                    aabb_pts.push_back(index2mean(id));
                }
            }
    
    // p_mean_lock_.clear();
}   

    /**
     * @brief 最远点采样 (FPS)
     * @param input_pts 原始点集
     * @param num_samples 需要采样的目标数量
     * @return 采样后的点集
     */
    std::vector<Eigen::Vector3d> SDFMap::farthestPointSampling(const std::vector<Eigen::Vector3d>& input_pts, int num_samples) {
        if (input_pts.size() <= num_samples) return input_pts;
        if (num_samples <= 0) return {};

        std::vector<Eigen::Vector3d> sampled_pts;
        sampled_pts.reserve(num_samples);

        // 记录每个点到已采样点集的最小距离
        std::vector<double> min_dist(input_pts.size(), std::numeric_limits<double>::max());

        // 1. 随机选择第一个点（或选择第一个）
        int curr_idx = 0;
        
        for (int i = 0; i < num_samples; ++i) {
            sampled_pts.push_back(input_pts[curr_idx]);

            double max_min_dist = -1.0;
            int next_idx = -1;

            // 2. 更新所有点到当前已采样点集的最小距离，并寻找下一个最远点
            for (int j = 0; j < input_pts.size(); ++j) {
                double d = (input_pts[j] - input_pts[curr_idx]).squaredNorm();
                if (d < min_dist[j]) {
                    min_dist[j] = d;
                }

                if (min_dist[j] > max_min_dist) {
                    max_min_dist = min_dist[j];
                    next_idx = j;
                }
            }
            curr_idx = next_idx;
        }

        return sampled_pts;
    }

void SDFMap::getAABBPointsSample(std::vector<Eigen::Vector3d>& aabb_pts, 
                const std::vector<Eigen::Vector3d>& sample_pts,
                const Eigen::Vector3d& box_size){
    aabb_pts.clear();

    if(sample_pts.size() == 0 ) {
        ROS_ERROR("getAABBPoints sample_pts.size() == 0");
        return;
    }

    resetVisBuffer();
    while (vis_lock_.test_and_set())
        ;


    for(size_t i=0; i<sample_pts.size(); ++i){

        // std::cout << "----- i: " << i << std::endl;
        // std::cout << "sample_pts: " << sample_pts[i].transpose() << std::endl;

        // find local min and max idx
        Eigen::Vector3d min_cut_p, max_cut_p;
        for(int j=0; j<3; ++j){
            min_cut_p(j) = sample_pts[i](j)-0.5*box_size(j);
            max_cut_p(j) = sample_pts[i](j)+0.5*box_size(j);
        }

        Eigen::Vector3i min_cut, max_cut;
        posToIndex(min_cut_p, min_cut);
        posToIndex(max_cut_p, max_cut);
        boundIndex(min_cut);
        boundIndex(max_cut);

        // std::cout << "min_cut: " << min_cut.transpose() << std::endl;
        // std::cout << "max_cut: " << max_cut.transpose() << std::endl;

        for(int x=min_cut(0); x<=max_cut(0); x++)
        for(int y=min_cut(1); y<=max_cut(1); y++)
        for(int z=min_cut(2); z<=max_cut(2); z++){
            Eigen::Vector3i id(x, y, z);
            if(!md_->vis_buffer_[toAddress(id)])
            {
                if(isOccupied(id))
                    // aabb_pts.push_back(idx2pos(id));
                    aabb_pts.push_back(index2mean(id));
            }
            md_->vis_buffer_[toAddress(x, y, z)] = true;
        }

    }

    vis_lock_.clear();



    // std::cout << "aabb_pts.size(): " << aabb_pts.size() << std::endl;

}

}  // namespace fast_planner
// SDFMap
