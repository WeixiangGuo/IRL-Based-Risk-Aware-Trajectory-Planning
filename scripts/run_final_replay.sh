#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# ============================================================
# AirGrasp MINCO-IRL: 复现最终 d1/d2 规划结果
# 使用方式：
#   zsh /home/gwx/airgrasp_ws/src/airgrasp_planning/airgrasp_minco_irl/scripts/run_final_replay.sh
# ============================================================

# ----------------------------
# 基础路径
# ----------------------------

WS_ROOT="/home/gwx/airgrasp_ws"                         # 工作空间根目录
PKG_ROOT="${WS_ROOT}/src/airgrasp_planning/airgrasp_minco_irl"  # 本项目根目录
CONDA_ENV="AirGrasp"                                    # Conda 环境名

# ----------------------------
# 需要改的核心参数
# ----------------------------

OPTIMIZER_RUN_DIR="${PKG_ROOT}/results/runs/irl_fd_manual_20260724_161550"  # 要复现的优化结果目录
RESULTS_DIR="${PKG_ROOT}/results/replays"                # replay 结果输出目录
RUN_NAME="final_d_replay_manual_$(date +%Y%m%d_%H%M%S)"  # 本次 replay 结果目录名

D_SOURCE="final"                                         # final 用最终 d；best 用历史最优 d
D1_OVERRIDE=""                                           # 留空则从优化结果读取
D2_OVERRIDE=""                                           # 留空则从优化结果读取

REPLAY_MODE="both"                                       # same_process / fresh / both
SAME_PROCESS_TRIALS="5"                                  # 同一个 planner 进程内重复次数
FRESH_TRIALS="3"                                         # 重启 planner 后重复次数

# ----------------------------
# 可选覆盖参数
# 留空表示继承 optimizer run 里的配置
# ----------------------------

DEMO_CSV=""                                              # 留空则继承原 demo
REFERENCE_CSV=""                                         # 留空则对比优化时 best trajectory
SCENE_CONFIG="${PKG_ROOT}/config/irl_221201_scene.yaml"  # replay 场景
WORLD_LAUNCH="${PKG_ROOT}/launch/irl_221201_pcd_replay_world.launch"  # PCD 背景场景
PLANNING_LAUNCH=""                                      # 留空则使用默认 planning launch

SPHERE_RADIUS=""                                         # UAM 球半径
OBSTACLES_INFLATION=""                                   # 地图障碍膨胀
SAFETY_ZONE_BUFFER=""                                    # 安全区显示/检查附加 buffer

RHO_ASTAR_WAYPOINT=""                                    # guide warm-start 权重
RHO_ASTAR_WAYPOINT_STAGE2=""                             # stage2 guide 权重
MINCO_WARM_START_CACHE=""                                # 是否缓存第一条 warm-start
MINCO_WARM_START_CACHE_FORCE_REUSE=""                    # 是否强制复用 warm-start

IRL_COLLISION_WEIGHT_SCALE=""                            # d1/d2 避障代价权重
IRL_OBS_OPTIMIZATION_MARGIN=""                           # J_obs 优化余量

S_GUIDE_ENABLE=""                                        # 是否启用 S-guide
S_GUIDE_BYPASS_ASTAR=""                                  # true 表示不用 A*
S_GUIDE_MODE=""                                          # fixed / adaptive
S_GUIDE_PATH=""                                          # 手动 guide；留空则继承
S_GUIDE_ENDPOINT_TOLERANCE=""                            # guide 起终点容差

# ----------------------------
# 质量评估参数
# 留空表示继承 optimizer run 里的 gate
# ----------------------------

TRAJECTORY_SELECTION="first"                             # first / best_quality
TRAJECTORY_COLLECTION_SEC="2.0"                          # 收集候选轨迹时间
QUALITY_MIN_CLEARANCE_1=""                               # 柱1最小 clearance 要求
QUALITY_MIN_CLEARANCE_2=""                               # 柱2最小 clearance 要求
QUALITY_MAX_PATH_LENGTH=""                               # 最大路径长度
QUALITY_MAX_LENGTH_RATIO=""                              # 路径长度/直线距离上限
QUALITY_MAX_LOCAL_TURN_DEG=""                            # 局部急转角上限
QUALITY_MAX_ARC_TURN_DEG=""                              # 弧段急转角上限
QUALITY_MAX_SELF_INTERSECTIONS=""                        # 自交数量上限

# ----------------------------
# 可视化 / 日志
# ----------------------------

USE_MINCO_FINAL_TRAJECTORY_VISUALIZER="true"             # 显示 replay 的 MINCO 最终轨迹
SHOW_MINCO_REFERENCE_TRAJECTORY="true"                   # 显示优化保存的 best/reference 轨迹
MINCO_FINAL_LINE_WIDTH="0.12"                            # 红色最终轨迹线宽
MINCO_REFERENCE_LINE_WIDTH="0.17"                        # 参考轨迹线宽
STREAM_PLANNING_DEBUG="false"                            # true 会打印 planner debug
STREAM_PLANNING_DEBUG_STRIDE="20"                        # debug 打印抽样间隔
TEE_STDOUT="false"                                       # true 会同步打印子进程日志
DRY_RUN="false"                                          # true 只生成命令不跑 ROS

# ----------------------------
# 超时参数
# ----------------------------

MASTER_TIMEOUT="20.0"
STARTUP_TIMEOUT="35.0"
MAP_TIMEOUT="20.0"
TRAJECTORY_TIMEOUT="60.0"
RESET_SETTLE_SEC="1.0"
CASE_SETTLE_SEC="2.0"
CLEANUP_SEC="2.0"
FIXED_ODOM_HZ="50.0"
START_TOLERANCE="0.05"
KEEP_LAUNCH_ON_FAILURE="false"

# ----------------------------
# 环境初始化
# ----------------------------

if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.zsh hook)"
else
  echo "[run_final_replay] 找不到 conda，请检查 Conda 安装路径。"
  exit 1
fi

conda activate "${CONDA_ENV}"

cd "${WS_ROOT}"
source devel/setup.zsh

# ----------------------------
# 启动 replay
# ----------------------------

cmd=(
  rosrun airgrasp_minco_irl run_final_d_replay.py
  --optimizer-run-dir "${OPTIMIZER_RUN_DIR}"
  --results-dir "${RESULTS_DIR}"
  --run-name "${RUN_NAME}"
  --d-source "${D_SOURCE}"
  --replay-mode "${REPLAY_MODE}"
  --same-process-trials "${SAME_PROCESS_TRIALS}"
  --fresh-trials "${FRESH_TRIALS}"
  --sample-count "200"
  --yaw-weight "0.0"
  --single-shot-script "${PKG_ROOT}/scripts/run_single_shot_planning_eval.py"
  --trajectory-selection "${TRAJECTORY_SELECTION}"
  --trajectory-collection-sec "${TRAJECTORY_COLLECTION_SEC}"
  --use-minco-final-trajectory-visualizer "${USE_MINCO_FINAL_TRAJECTORY_VISUALIZER}"
  --minco-final-line-width "${MINCO_FINAL_LINE_WIDTH}"
  --minco-reference-line-width "${MINCO_REFERENCE_LINE_WIDTH}"
  --stream-planning-debug "${STREAM_PLANNING_DEBUG}"
  --stream-planning-debug-stride "${STREAM_PLANNING_DEBUG_STRIDE}"
  --tee-stdout "${TEE_STDOUT}"
  --master-timeout "${MASTER_TIMEOUT}"
  --startup-timeout "${STARTUP_TIMEOUT}"
  --map-timeout "${MAP_TIMEOUT}"
  --trajectory-timeout "${TRAJECTORY_TIMEOUT}"
  --reset-settle-sec "${RESET_SETTLE_SEC}"
  --case-settle-sec "${CASE_SETTLE_SEC}"
  --cleanup-sec "${CLEANUP_SEC}"
  --fixed-odom-hz "${FIXED_ODOM_HZ}"
  --start-tolerance "${START_TOLERANCE}"
)

if [[ -n "${D1_OVERRIDE}" ]]; then cmd+=("--d1" "${D1_OVERRIDE}"); fi
if [[ -n "${D2_OVERRIDE}" ]]; then cmd+=("--d2" "${D2_OVERRIDE}"); fi
if [[ -n "${DEMO_CSV}" ]]; then cmd+=("--demo-csv" "${DEMO_CSV}"); fi
if [[ -n "${REFERENCE_CSV}" ]]; then cmd+=("--reference-csv" "${REFERENCE_CSV}"); fi
if [[ -n "${SCENE_CONFIG}" ]]; then cmd+=("--scene-config" "${SCENE_CONFIG}"); fi
if [[ -n "${WORLD_LAUNCH}" ]]; then cmd+=("--world-launch" "${WORLD_LAUNCH}"); fi
if [[ -n "${PLANNING_LAUNCH}" ]]; then cmd+=("--planning-launch" "${PLANNING_LAUNCH}"); fi
if [[ -n "${SPHERE_RADIUS}" ]]; then cmd+=("--sphere-radius" "${SPHERE_RADIUS}"); fi
if [[ -n "${OBSTACLES_INFLATION}" ]]; then cmd+=("--obstacles-inflation" "${OBSTACLES_INFLATION}"); fi
if [[ -n "${SAFETY_ZONE_BUFFER}" ]]; then cmd+=("--safety-zone-buffer" "${SAFETY_ZONE_BUFFER}"); fi
if [[ -n "${RHO_ASTAR_WAYPOINT}" ]]; then cmd+=("--rho-astar-waypoint" "${RHO_ASTAR_WAYPOINT}"); fi
if [[ -n "${RHO_ASTAR_WAYPOINT_STAGE2}" ]]; then cmd+=("--rho-astar-waypoint-stage2" "${RHO_ASTAR_WAYPOINT_STAGE2}"); fi
if [[ -n "${MINCO_WARM_START_CACHE}" ]]; then cmd+=("--minco-warm-start-cache" "${MINCO_WARM_START_CACHE}"); fi
if [[ -n "${MINCO_WARM_START_CACHE_FORCE_REUSE}" ]]; then cmd+=("--minco-warm-start-cache-force-reuse" "${MINCO_WARM_START_CACHE_FORCE_REUSE}"); fi
if [[ -n "${IRL_COLLISION_WEIGHT_SCALE}" ]]; then cmd+=("--irl-collision-weight-scale" "${IRL_COLLISION_WEIGHT_SCALE}"); fi
if [[ -n "${IRL_OBS_OPTIMIZATION_MARGIN}" ]]; then cmd+=("--irl-obs-optimization-margin" "${IRL_OBS_OPTIMIZATION_MARGIN}"); fi
if [[ -n "${S_GUIDE_ENABLE}" ]]; then cmd+=("--s-guide-enable" "${S_GUIDE_ENABLE}"); fi
if [[ -n "${S_GUIDE_BYPASS_ASTAR}" ]]; then cmd+=("--s-guide-bypass-astar" "${S_GUIDE_BYPASS_ASTAR}"); fi
if [[ -n "${S_GUIDE_MODE}" ]]; then cmd+=("--s-guide-mode" "${S_GUIDE_MODE}"); fi
if [[ -n "${S_GUIDE_ENDPOINT_TOLERANCE}" ]]; then cmd+=("--s-guide-endpoint-tolerance" "${S_GUIDE_ENDPOINT_TOLERANCE}"); fi
if [[ -n "${S_GUIDE_PATH}" ]]; then cmd+=("--s-guide-path=${S_GUIDE_PATH}"); fi
if [[ -n "${QUALITY_MIN_CLEARANCE_1}" ]]; then cmd+=("--quality-min-clearance-1" "${QUALITY_MIN_CLEARANCE_1}"); fi
if [[ -n "${QUALITY_MIN_CLEARANCE_2}" ]]; then cmd+=("--quality-min-clearance-2" "${QUALITY_MIN_CLEARANCE_2}"); fi
if [[ -n "${QUALITY_MAX_PATH_LENGTH}" ]]; then cmd+=("--quality-max-path-length" "${QUALITY_MAX_PATH_LENGTH}"); fi
if [[ -n "${QUALITY_MAX_LENGTH_RATIO}" ]]; then cmd+=("--quality-max-length-ratio" "${QUALITY_MAX_LENGTH_RATIO}"); fi
if [[ -n "${QUALITY_MAX_LOCAL_TURN_DEG}" ]]; then cmd+=("--quality-max-local-turn-deg" "${QUALITY_MAX_LOCAL_TURN_DEG}"); fi
if [[ -n "${QUALITY_MAX_ARC_TURN_DEG}" ]]; then cmd+=("--quality-max-arc-turn-deg" "${QUALITY_MAX_ARC_TURN_DEG}"); fi
if [[ -n "${QUALITY_MAX_SELF_INTERSECTIONS}" ]]; then cmd+=("--quality-max-self-intersections" "${QUALITY_MAX_SELF_INTERSECTIONS}"); fi
if [[ "${KEEP_LAUNCH_ON_FAILURE}" == "true" ]]; then cmd+=("--keep-launch-on-failure"); fi
if [[ "${DRY_RUN}" == "true" ]]; then cmd+=("--dry-run"); fi
cmd+=("--show-minco-reference-trajectory" "${SHOW_MINCO_REFERENCE_TRAJECTORY}")

echo "[run_final_replay] 即将执行："
printf '  %q' "${cmd[@]}"
echo

"${cmd[@]}"
