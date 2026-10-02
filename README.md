# AirGrasp MINCO-IRL

面向空中机械臂的语义风险感知轨迹规划：从人类飞行示范中反演不同障碍物的安全距离，并通过 MINCO 生成满足动力学与碰撞约束的可执行轨迹。

本仓库是独立整理的 IRL 项目代码，不是 AirGrasp 总工程的镜像。它包含 IRL 外层优化、ROS 场景与评估工具、实飞示范数据、参考实验结果，以及规划器侧的 MINCO/IRL C++ 核心实现。

## 方法概览

- 输入：人类示范轨迹、障碍物语义分组和飞行场景。
- 外层 IRL：搜索两类障碍物的安全距离 `d1/d2`，最小化示范轨迹与规划轨迹之间的模仿损失。
- 内层规划：使用两阶段 L-BFGS/MINCO 优化碰撞、平滑性、飞行时间、姿态和机械臂状态。
- 安全机制：优化过程中加入解析圆柱碰撞代价，并在输出轨迹上执行独立的安全裕度硬检查。
- 执行链路：支持仿真评估、重复回放和带人工确认门的 onboard 执行。

参考实飞示范对应的归档优化结果位于 `results/runs/irl_fd_manual_20260724_161550/`；该次运行得到 `d1=0.01 m`、`d2=0.22 m`，并通过最终轨迹可行性检查。

## 仓库结构

```text
config/             IRL 场景、双柱场景与 S-guide 配置
data/               仿真/实飞示范轨迹与点云地图
launch/             ROS 场景、规划和实机可视化入口
scripts/            数据采集、外层优化、回放和 onboard 执行
src/                 IRL 外层优化器与自适应 guide 实现
results/             参考优化与重复性实验结果
planner_overlay/     MINCO/IRL 规划器侧 C++ 核心文件
```

`planner_overlay/` 按原 AirGrasp 工作区相对路径保存规划器侧实现。将其合并进兼容的 `airgrasp_planning` 工作区后，本 ROS 包才能构成完整的内外层优化链路；具体文件和集成方式见 `planner_overlay/README.md`。

## MINCO 轨迹生成数据流

整体链路可以按下面理解：

```text
two_pillar_scene.yaml
        |
        v
two_pillar_world.launch
  -> two_pillar_clouds.py
  -> /pcl_from_pcd/env_pcl, /pcl_from_pcd/tgt_pcl
  -> bridge_node/pcl_sync_world_sim.launch
  -> /sync_frame_pcl_world
        |
        v
two_pillar_planning.launch
  -> /drone0/planning/planning
  -> MapInterface: env_map/tgt_map + IRL obstacle groups
        |
        +<-- /drone0/odom
        +<-- /joint_state_est_sim
        +<-- /drone0/planning/uam_state_goal_cmd
        +<-- /drone0/planning/irl_safety_distances
        +<-- /scene/planner_hold
        |
        v
TLPlanner::plan_goal()
  -> A* path or S-guide path
  -> TrajOpt::generate_traj_clutter()
  -> two-stage LBFGS MINCO optimization
        |
        v
ShareDataManager::traj_info_
        |
        v
TrajServer
  -> /position_cmd
  -> /drone0/planning/planned_position_command_trajectory
  -> /drone0/planning/planned_minco_traj
  -> /drone0/planning/planned_terminal_state
```

## 关键输入

### 场景配置

主配置文件是 `config/two_pillar_scene.yaml`：

- `pillar1_center_xy`, `pillar2_center_xy`, `pillar_radius`, `pillar_z_min`, `pillar_z_max` 定义两根柱子。
- `start_position`, `goal_position`, `start_yaw`, `goal_yaw` 定义 single-shot 起终点。
- `reference_path` 定义当前 one-shot 评估使用的固定 S-guide/reference path。
- `start_theta`, `goal_theta`, `start_dtheta`, `goal_dtheta` 定义机械臂关节边界状态。

### 点云和地图

`launch/two_pillar_world.launch` 启动 `scripts/two_pillar_clouds.py` 和 `bridge_node`：

- `/pcl_from_pcd/env_pcl`: 环境点云。
- `/pcl_from_pcd/tgt_pcl`: 目标点云/同步触发点云。
- `/sync_frame_pcl_world`: `bridge_node` 把缓存的 env/tgt 点云打包成 `quadrotor_msgs/SyncFrame`。

当前 launch 默认 `publish_point_clouds=false`，因此两柱可作为 marker/同步触发场景运行；若需要让 PCL 真正带障碍物，启动 world 时传 `publish_point_clouds:=true`。此时本包约定两根柱子都进入 `env_pcl`，`tgt_pcl` 为空触发云。

`two_pillar_planning.launch` 中 planner 侧相关参数：

- `use_world_pcl=true`: `MapInterface` 订阅 `/sync_frame_pcl_world`。
- `clear_tgt_from_env_map=false`, `merge_tgt_into_env_map=false`: env/tgt 地图保持分离。
- `irl_groups_from_world_pcl=true`: 允许从 world PCL 缓存 IRL 障碍组。
- `irl_split_env_pcl_by_x=true`, `irl_split_x=0.0`: `x < 0` 为 obstacle group 1，使用 `d1`；`x >= 0` 为 group 2，使用 `d2`。
- `irl_analytic_cylinder_enable=true`: 默认使用解析圆柱碰撞项；此时即使点云为空，优化器也会按 launch 中的圆柱参数计算 IRL 碰撞代价和硬检查。

### 当前状态和目标状态

planner 节点运行在私有命名空间 `/drone0/planning`。主要输入 topic：

| Topic | 类型 | 作用 |
| --- | --- | --- |
| `/drone0/odom` | `nav_msgs/Odometry` | UAV 当前位姿、速度。 |
| `/joint_state_est_sim` | `sensor_msgs/JointState` | 机械臂当前关节角/角速度。 |
| `/drone0/planning/uam_state_goal_cmd` | `quadrotor_msgs/UAMFullState` | 目标 UAV 位姿和目标关节状态。 |
| `/scene/planner_hold` | `std_msgs/Bool` | 为 true 时 planner 保持 HOVER，释放后下一条 goal 可以触发规划。 |
| `/drone0/planning/irl_safety_distances` | `geometry_msgs/Vector3` | 运行时写入 IRL 安全距离：`x=d1`, `y=d2`, `z=sphere_radius`。`z <= 0` 表示保留当前半径。 |

`DataCallBacks::goal_full_state_callback()` 会把 `UAMFullState` 转为内部 `Odom goal`，包括目标位置、yaw、`theta` 和 `dtheta`。`Planner::irl_safety_distances_cb()` 会把 `d1/d2/radius` 写入 `TrajOpt::set_irl_sphere_safety_distances()`。

## 规划内部流

### 1. Planner 取状态

`Planner::plan_thread()` 每个规划周期读取：

- 当前 `odom_data`；
- 当前 `theta_cur_`；
- 最新 `goal_data`；
- 是否收到地图 `/sdf_map/esdf`；
- 上一条轨迹是否可作为 replan 初值。

如果没有有效上一条轨迹，或者 `/scene/planner_hold=true` 后强制重置，则从当前 odom 开始规划。

### 2. TLPlanner 生成几何路径

`TLPlanner::plan_goal()` 把输入整理成 MINCO 边界条件：

- `init_state`: 位置、速度、加速度、jerk；
- `final_state`: 终点位置和目标速度；
- `init_yaw`, `final_yaw`;
- `init_thetas`, `final_thetas`。

然后生成 `path3d`：

- 默认可走 A*：`envPtr_->short_astar(start, goal, path3d)`。
- 若 `s_guide_path_enable=true` 且 `s_guide_bypass_astar=true`，直接用 S-guide 替代 A*。
- 若只开启 `s_guide_path_enable=true`，A* 成功后仍会用 S-guide 覆盖 `path3d`，并发布可视化路径 `s_guide`。

两柱实验的 single-shot 采集默认开启：

```text
s_guide_enable=true
s_guide_bypass_astar=true
s_guide_endpoint_tolerance=0.45
```

### 3. TrajOpt 生成 MINCO 轨迹

`TrajOpt::generate_traj_clutter()` 是 MINCO 生成核心，当前调用的是 7 维状态轨迹：

```cpp
trajoptPtr_->generate_traj_clutter(
    init_state, final_state,
    init_yaw, final_yaw,
    init_thetas, final_thetas,
    0.8, path3d, traj_goal);
```

内部主要步骤：

1. 根据 `path3d` 抽取中间点 `mid_q_vec`，确定 MINCO piece 数 `N_`。
2. 初始化优化变量：每段时间 `T`、中间控制点 `P`、yaw 中间值、theta 中间值。
3. 初始化 `minco_s4_opt_`、`minco_s2_yaw_opt_` 和各关节 `minco_s2_theta_opt_vec_`。
4. 调 `gridmapPtr_->getAABBPoints(...)` 得到两组障碍采样点；若开启解析圆柱，则优化碰撞项直接使用圆柱参数。
5. 两阶段 L-BFGS：
   - `stage1_no_sdf`: warm start，不加碰撞梯度；
   - `stage2_with_sdf`: 加 IRL sphere/cylinder 碰撞梯度、S-guide tracking、yaw/theta/动力学等代价。
6. 生成 `Trajectory<7>`，并用 `is_irl_sphere_traj_valid()` 做最终硬检查。

IRL 安全模型把 UAV 近似成一个球：

```text
required_clearance_1 = sphere_radius + d1
required_clearance_2 = sphere_radius + d2
```

若轨迹采样点到 obstacle group 的最小 margin 小于阈值，会被拒绝，日志中通常出现：

```text
[GenTraj] reject trajectory by IRL sphere hard check
[MINCO-IRL] stage=final ... status=fail_or_reject
```

## 轨迹输出

规划成功后，`Planner::plan_thread()` 把 `traj_now` 写入 `ShareDataManager::traj_info_`。`TrajServer::cmd_thread()` 读取该轨迹并发布：

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | 控制器实时跟踪的当前采样命令。 |
| `/drone0/planning/planned_position_command_trajectory` | `quadrotor_msgs/PositionCommandTrajectory` | 按 `cmd_hz` 采样后的完整轨迹点序列，评估脚本主要读取这个。 |
| `/drone0/planning/planned_minco_traj` | `quadrotor_msgs/MincoTraj` | 原始 MINCO piece duration 和多项式系数。 |
| `/drone0/planning/planned_terminal_state` | `quadrotor_msgs/UAMFullState` | 轨迹末端 UAV/关节状态。 |
| `/drone0/planning/planner_failure` | `std_msgs/Header` | 规划失败原因，原因写在 `frame_id`。 |

`PositionCommandTrajectory` 是采样数据：

```text
trajectory_id
sample_dt
total_duration
PositionCommand[] points
```

`MincoTraj` 是多项式数据：

```text
traj_id, start_time, state, traj_type, order
duration[]
coef_x[], coef_y[], coef_z[], coef_psi[]
theta_dof, coef_theta[]
```

调试时要区分二者：CSV demo 默认来自采样轨迹，不是 MINCO 系数本身。

## Demo 和评估数据流

### single-shot 采集

入口：

```bash
python3 src/airgrasp_planning/airgrasp_minco_irl/scripts/collect_sim_demo.py \
  --d1 0.50 \
  --d2 0.01
```

`collect_sim_demo.py` 会调用 `run_single_shot_planning_eval.py`：

1. 启动 `two_pillar_world.launch` 和 `two_pillar_planning.launch`。
2. 固定发布起点 `/drone0/odom` 和 `/joint_state_est_sim`。
3. 发布 `/drone0/planning/irl_safety_distances = (d1, d2, sphere_radius)`。
4. 发布一次 `/drone0/planning/uam_state_goal_cmd`。
5. 释放 `/scene/planner_hold`。
6. 等待第一条 `/drone0/planning/planned_position_command_trajectory`。
7. 写出 case 结果。

输出目录位于：

```text
airgrasp_minco_irl/results/scratch/<run_name>/
```

主要文件：

- `summary.csv`, `summary.json`: 每个 `d1/d2` case 的统计。
- `run_metadata.json`: 本次评估的配置。
- `case_XX_d1_..._d2_.../planned_trajectories.csv`: 收到的所有采样轨迹。
- `case_XX_d1_..._d2_.../candidate_planned_path.csv`: single-shot 选择的候选轨迹。
- `case_XX_d1_..._d2_.../case.json`: case 元信息、失败原因和轨迹摘要。
- `planning.log`, `world.log`: 对应 launch 的完整日志。
- `demo_trajectory.csv`: `collect_sim_demo.py` 从 `candidate_planned_path.csv` 复制出的最终 demo。
- `demo_metadata.json`: demo 源文件、`d1/d2/radius`、case 路径、summary 的索引信息。

## 常用启动方式

只启动两柱世界和 planner，配 mock follower/轨迹历史可视化：

```bash
roslaunch airgrasp_minco_irl two_pillar_planner_only.launch
```

启动完整仿真、显示、控制器：

```bash
roslaunch airgrasp_minco_irl two_pillar_sim.launch
```

直接跑 single-shot 评估：

```bash
python3 src/airgrasp_planning/airgrasp_minco_irl/scripts/run_single_shot_planning_eval.py \
  --cases 0.50,0.01 \
  --s-guide-enable true \
  --s-guide-bypass-astar true
```

## 排查入口

- 没有轨迹输出：检查 `/drone0/odom`、`/joint_state_est_sim`、`/drone0/planning/uam_state_goal_cmd` 是否有 publisher/subscriber，且 `/scene/planner_hold` 已释放。
- 地图没准备好：检查 `/sync_frame_pcl_world` 和 `/sdf_map/esdf`。
- `d1/d2` 没生效：看 `planning.log` 是否有 `[TrajOpt][IRL] sphere collision d1=... d2=... radius=...`。
- 轨迹被拒绝：看 `[MINCO-IRL] stage=final` 和 `[GenTraj] reject trajectory by IRL sphere hard check`。
- 需要看 MINCO 系数：订阅 `/drone0/planning/planned_minco_traj`，不要只看 `demo_trajectory.csv`。
