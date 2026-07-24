#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# ============================================================
# AirGrasp MINCO-IRL: 跑 d1/d2 外层优化
# 使用方式：
#   zsh /home/gwx/airgrasp_ws/src/airgrasp_planning/airgrasp_minco_irl/scripts/run_irl_optimization.sh
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

DEMO_CSV="${PKG_ROOT}/data/demos/real_world_human_flight/irl_221201_raw_world/demo_trajectory.csv"  # 要学习的 demo
RUN_NAME="irl_fd_manual_$(date +%Y%m%d_%H%M%S)"          # 本次优化结果目录名

INIT_D1="0.01"                                           # d1 初值
INIT_D2="0.10"                                           # d2 初值
MAX_ITERS="12"                                            # 优化轮数
EPS="0.015"                                              # 有限差分步长
LEARNING_RATE="0.30"                                     # 梯度模式步长；best_neighbor 模式下影响较小
MAX_STEP="0.03"                                          # 每轮 d 最大变化量
MIN_D="0.005"                                            # d 下界
MAX_D="0.80"                                             # d 上界

# ----------------------------
# planner / guide 参数
# ----------------------------

SCENE_CONFIG="${PKG_ROOT}/config/irl_221201_scene.yaml"  # 场景配置
RESULTS_DIR="${PKG_ROOT}/results/runs"                   # 优化结果输出目录

S_GUIDE_ENABLE="true"                                    # 是否启用 S-guide
S_GUIDE_BYPASS_ASTAR="true"                              # true 表示不用 A*
S_GUIDE_MODE="fixed"                                     # fixed 使用 scene/reference path
S_GUIDE_PATH=""                                          # 留空则用场景 reference_path
S_GUIDE_ENDPOINT_TOLERANCE="0.45"                        # guide 起终点容差

RHO_ASTAR_WAYPOINT="100000"                              # guide warm-start 权重
RHO_ASTAR_WAYPOINT_STAGE2="0.0"                          # stage2 guide 权重，0 表示后端不强拉 guide
RHO_ASTAR_WAYPOINT_SCHEDULE=""                           # 需要逐轮变化时填 "100000,70000,50000"
ENABLE_TAIL_CONSTRAINT="false"                           # false 表示不强制末端 tail 对齐

MINCO_WARM_START_CACHE="true"                            # 缓存第一条 warm-start
MINCO_WARM_START_CACHE_FORCE_REUSE="true"                 # 后续 case 强制复用 warm-start

IRL_COLLISION_WEIGHT_SCALE="5.0"                         # d1/d2 避障代价权重
IRL_OBS_OPTIMIZATION_MARGIN="0.02"                       # J_obs 优化余量
OBSTACLES_INFLATION="0.40"                               # 地图障碍膨胀
SPHERE_RADIUS="0.25"                                     # UAM 球半径

# ----------------------------
# loss / 更新策略
# ----------------------------

OUTER_UPDATE_MODE="best_neighbor"                        # 推荐：选当前/plus/minus 中 loss 最小者
SAMPLE_COUNT="200"                                       # loss 轨迹采样点数
YAW_WEIGHT="0.0"                                         # yaw loss 权重；目前只学位置
FAILURE_LOSS_MODE="feasibility_boundary"                 # 失败 case 用 margin 转 feasibility loss
FEASIBILITY_LOSS_WEIGHT="300.0"                          # feasibility loss 权重
FAILURE_STATUS_PENALTY="0.02"                            # 失败附加小惩罚

# ----------------------------
# 可视化 / 日志
# ----------------------------

DISABLE_VISUALIZATION="false"                            # true 则关闭优化过程 RViz marker
VISUALIZE_MINCO_TRAJECTORIES="true"                      # 显示每次 MINCO 输出轨迹
VIZ_MINCO_HISTORY_LIMIT="300"                            # RViz 最多保留多少条轨迹
VIZ_STRIDE="3"                                           # 轨迹采样可视化间隔
HOLD_AFTER_SEC="8.0"                                     # 优化结束后 marker 保持时间
OPTIMIZER_LOG_MODE="concise"                             # concise 简洁日志
STREAM_MINCO_DEBUG="false"                               # true 会打印 MINCO debug 行
STREAM_MINCO_DEBUG_STRIDE="20"                           # debug 打印抽样间隔

# ----------------------------
# 超时参数
# ----------------------------

MASTER_TIMEOUT="20.0"
STARTUP_TIMEOUT="35.0"
MAP_TIMEOUT="20.0"
GOAL_TIMEOUT="90.0"
TRAJECTORY_TIMEOUT="45.0"
RESET_SETTLE_SEC="1.0"
CASE_SETTLE_SEC="2.0"
CLEANUP_SEC="2.0"

# ----------------------------
# 环境初始化
# ----------------------------

if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.zsh hook)"
else
  echo "[run_irl_optimization] 找不到 conda，请检查 Conda 安装路径。"
  exit 1
fi

conda activate "${CONDA_ENV}"

cd "${WS_ROOT}"
source devel/setup.zsh

# ----------------------------
# 启动优化
# ----------------------------

cmd=(
  rosrun airgrasp_minco_irl run_fd_irl_optimizer.py
  --demo-csv "${DEMO_CSV}"
  --results-dir "${RESULTS_DIR}"
  --run-name "${RUN_NAME}"
  --scene-config "${SCENE_CONFIG}"
  --eval-mode single_shot
  --persistent-planner true
  --max-iters "${MAX_ITERS}"
  --init-d1 "${INIT_D1}"
  --init-d2 "${INIT_D2}"
  --eps "${EPS}"
  --learning-rate "${LEARNING_RATE}"
  --max-step "${MAX_STEP}"
  --min-d "${MIN_D}"
  --max-d "${MAX_D}"
  --outer-update-mode "${OUTER_UPDATE_MODE}"
  --sample-count "${SAMPLE_COUNT}"
  --yaw-weight "${YAW_WEIGHT}"
  --candidate-source planned
  --planned-selection first_by_goal
  --failure-loss-mode "${FAILURE_LOSS_MODE}"
  --feasibility-loss-weight "${FEASIBILITY_LOSS_WEIGHT}"
  --failure-status-penalty "${FAILURE_STATUS_PENALTY}"
  --sphere-radius "${SPHERE_RADIUS}"
  --obstacles-inflation "${OBSTACLES_INFLATION}"
  --rho-astar-waypoint "${RHO_ASTAR_WAYPOINT}"
  --rho-astar-waypoint-stage2 "${RHO_ASTAR_WAYPOINT_STAGE2}"
  --rho-astar-waypoint-schedule "${RHO_ASTAR_WAYPOINT_SCHEDULE}"
  --enable-tail-constraint "${ENABLE_TAIL_CONSTRAINT}"
  --minco-warm-start-cache "${MINCO_WARM_START_CACHE}"
  --minco-warm-start-cache-force-reuse "${MINCO_WARM_START_CACHE_FORCE_REUSE}"
  --irl-collision-weight-scale "${IRL_COLLISION_WEIGHT_SCALE}"
  --irl-obs-optimization-margin "${IRL_OBS_OPTIMIZATION_MARGIN}"
  --s-guide-enable "${S_GUIDE_ENABLE}"
  --s-guide-bypass-astar "${S_GUIDE_BYPASS_ASTAR}"
  --s-guide-mode "${S_GUIDE_MODE}"
  --s-guide-endpoint-tolerance "${S_GUIDE_ENDPOINT_TOLERANCE}"
  --visualize-minco-trajectories "${VISUALIZE_MINCO_TRAJECTORIES}"
  --viz-minco-history-limit "${VIZ_MINCO_HISTORY_LIMIT}"
  --viz-stride "${VIZ_STRIDE}"
  --hold-after-sec "${HOLD_AFTER_SEC}"
  --optimizer-log-mode "${OPTIMIZER_LOG_MODE}"
  --stream-minco-debug "${STREAM_MINCO_DEBUG}"
  --stream-minco-debug-stride "${STREAM_MINCO_DEBUG_STRIDE}"
  --master-timeout "${MASTER_TIMEOUT}"
  --startup-timeout "${STARTUP_TIMEOUT}"
  --map-timeout "${MAP_TIMEOUT}"
  --goal-timeout "${GOAL_TIMEOUT}"
  --trajectory-timeout "${TRAJECTORY_TIMEOUT}"
  --reset-settle-sec "${RESET_SETTLE_SEC}"
  --case-settle-sec "${CASE_SETTLE_SEC}"
  --cleanup-sec "${CLEANUP_SEC}"
)

if [[ -n "${S_GUIDE_PATH}" ]]; then
  cmd+=("--s-guide-path=${S_GUIDE_PATH}")
fi

if [[ "${DISABLE_VISUALIZATION}" == "true" ]]; then
  cmd+=("--disable-visualization")
fi

echo "[run_irl_optimization] 即将执行："
printf '  %q' "${cmd[@]}"
echo

"${cmd[@]}"
