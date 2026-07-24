#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# ============================================================
# AirGrasp MINCO-IRL: 采集 one-shot demo 专用启动脚本
# 使用方式：
#   zsh /home/gwx/airgrasp_ws/src/airgrasp_planning/airgrasp_minco_irl/scripts/run_collect_demo.sh
#
# 说明：
#   只需要改下面“可调参数区”的变量，不需要再手打一长串命令。
# ============================================================

# ----------------------------
# 基础环境
# ----------------------------

# 工作空间根目录：一般不需要改。
WS_ROOT="/home/gwx/airgrasp_ws"

# 本项目根目录：一般不需要改。
PKG_ROOT="${WS_ROOT}/src/airgrasp_planning/airgrasp_minco_irl"

# Conda 环境名：你的工程目前使用 AirGrasp。
CONDA_ENV="AirGrasp"

# ----------------------------
# 可调参数区：demo 采集核心参数
# ----------------------------

# d1/d2：传给 MINCO-IRL 的两根立柱安全距离参数。
# 注意：这里的 d 是优化器/避障代价使用的安全阈值，不一定等于最终 demo 的真实 clearance。
D1="0.08"
D2="0.55"

# UAV 球模型半径：当前我们把完整 UAM 近似成一个球。
SPHERE_RADIUS="0.25"

# 场景配置文件：定义两根柱子、起点、终点、reference_path 等。
SCENE_CONFIG="${PKG_ROOT}/config/two_pillar_scene.yaml"

# demo 输出目录：仿真生成的 demo 统一保存到 simulated_flight。
RESULTS_DIR="${PKG_ROOT}/data/demos/simulated_flight"

# 本次 demo 目录名。
# 设为空字符串 "" 时，collect_sim_demo.py 会自动生成 sim_demo_d1_xxx_d2_xxx_时间戳。
# 如果你想固定命名，可以改成例如：
# RUN_NAME="one_shot_demo_c1_010_c2_070_manual_$(date +%Y%m%d_%H%M%S)"
RUN_NAME="one_shot_demo_manual_$(date +%Y%m%d_%H%M%S)"

# ----------------------------
# 可调参数区：guide / MINCO 权重
# ----------------------------

# 是否启用 S-guide：true 表示不用 A*，用我们给的 guide/reference path 提供 S 型拓扑。
S_GUIDE_ENABLE="true"

# 是否绕过 A*：true 表示直接使用 S-guide 作为前端路径。
S_GUIDE_BYPASS_ASTAR="true"

# guide 模式：
#   fixed    使用场景配置中的 reference_path 或手动传入的 S_GUIDE_PATH。
#   adaptive 根据 d1/d2 和圆柱位置动态生成 guide。
S_GUIDE_MODE="fixed"

# 手动 guide 路径：
#   留空 "" 时，默认使用 two_pillar_scene.yaml 里的 reference_path。
#   如果要手动给 guide，格式必须是：
#   "x,y,z;x,y,z;x,y,z"
S_GUIDE_PATH=""

# 起终点容差：guide 首尾和场景起终点的允许误差。
S_GUIDE_ENDPOINT_TOLERANCE="0.45"

# rhoAstarWaypoint：guide tracking 权重。
# 当前 demo 采集建议给大一点，保证 one-shot 轨迹拓扑稳定。
RHO_ASTAR_WAYPOINT="100000"

# stage2 的 guide tracking 权重：
# 当前逻辑下设成 0.0，表示后端阶段不再持续强拉 guide。
RHO_ASTAR_WAYPOINT_STAGE2="0.0"

# 是否启用末端 tail constraint：
# false 表示轨迹只到目标点，不强制末端按 goal_yaw 拉出一小段尾巴。
ENABLE_TAIL_CONSTRAINT="false"

# 是否启用 MINCO warm-start cache：demo 采集一般不需要缓存。
MINCO_WARM_START_CACHE="false"
MINCO_WARM_START_CACHE_FORCE_REUSE="false"

# 避障代价权重倍率：越大越重视 d1/d2 安全距离。
IRL_COLLISION_WEIGHT_SCALE="5.0"

# 后端优化余量：J_obs 用 d + margin 做优化，hard check 仍按 d。
IRL_OBS_OPTIMIZATION_MARGIN="0.05"

# 前端地图障碍膨胀：影响传统地图/前端碰撞判断，不是 d1/d2 本身。
OBSTACLES_INFLATION="0.40"

# 只影响安全区可视化/解析检查额外半径；一般保持 0。
SAFETY_ZONE_BUFFER="0.0"

# ----------------------------
# 可调参数区：轨迹质量筛选
# ----------------------------

# 轨迹选择策略：
#   first        直接取第一条规划轨迹。
#   best_quality 收集一小段时间内的轨迹，选质量最好的轨迹。
TRAJECTORY_SELECTION="best_quality"

# 采集规划轨迹的时间窗口，单位秒。
TRAJECTORY_COLLECTION_SEC="3.0"

# demo 质量门限：期望柱 1 / 柱 2 的最小 clearance 下界。
# 当前默认值来自我们保留下来的 v5 demo 采集设置。
QUALITY_MIN_CLEARANCE_1="0.09"
QUALITY_MIN_CLEARANCE_2="0.68"

# 路径长度和弯曲质量门限，防止急转、打结、自交。
QUALITY_MAX_PATH_LENGTH="12.5"
QUALITY_MAX_LENGTH_RATIO="1.8"
QUALITY_MAX_LOCAL_TURN_DEG="35.0"
QUALITY_MAX_ARC_TURN_DEG="55.0"
QUALITY_MAX_SELF_INTERSECTIONS="0"

# 曲率/自交检查采样参数：一般不需要改。
QUALITY_ARC_TURN_SPACING="0.05"
QUALITY_ARC_TURN_HALF_WINDOW_M="0.10"
QUALITY_SAMPLE_STRIDE="10"
QUALITY_SELF_INTERSECTION_MAX_POINTS="220"

# ----------------------------
# 可调参数区：超时参数
# ----------------------------

MASTER_TIMEOUT="20.0"
STARTUP_TIMEOUT="35.0"
MAP_TIMEOUT="20.0"
TRAJECTORY_TIMEOUT="45.0"
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
  echo "[run_collect_demo] 找不到 conda，请检查 Conda 安装路径。"
  exit 1
fi

conda activate "${CONDA_ENV}"

cd "${WS_ROOT}"
source devel/setup.zsh

# ----------------------------
# 启动采集
# ----------------------------

cmd=(
  python3 "${PKG_ROOT}/scripts/collect_sim_demo.py"
  --d1 "${D1}"
  --d2 "${D2}"
  --sphere-radius "${SPHERE_RADIUS}"
  --scene-config "${SCENE_CONFIG}"
  --results-dir "${RESULTS_DIR}"
  --run-name "${RUN_NAME}"
  --s-guide-enable "${S_GUIDE_ENABLE}"
  --s-guide-bypass-astar "${S_GUIDE_BYPASS_ASTAR}"
  --s-guide-mode "${S_GUIDE_MODE}"
  --s-guide-endpoint-tolerance "${S_GUIDE_ENDPOINT_TOLERANCE}"
  --rho-astar-waypoint "${RHO_ASTAR_WAYPOINT}"
  --rho-astar-waypoint-stage2 "${RHO_ASTAR_WAYPOINT_STAGE2}"
  --enable-tail-constraint "${ENABLE_TAIL_CONSTRAINT}"
  --minco-warm-start-cache "${MINCO_WARM_START_CACHE}"
  --minco-warm-start-cache-force-reuse "${MINCO_WARM_START_CACHE_FORCE_REUSE}"
  --irl-collision-weight-scale "${IRL_COLLISION_WEIGHT_SCALE}"
  --irl-obs-optimization-margin "${IRL_OBS_OPTIMIZATION_MARGIN}"
  --obstacles-inflation "${OBSTACLES_INFLATION}"
  --safety-zone-buffer "${SAFETY_ZONE_BUFFER}"
  --trajectory-selection "${TRAJECTORY_SELECTION}"
  --trajectory-collection-sec "${TRAJECTORY_COLLECTION_SEC}"
  --quality-min-clearance-1 "${QUALITY_MIN_CLEARANCE_1}"
  --quality-min-clearance-2 "${QUALITY_MIN_CLEARANCE_2}"
  --quality-max-path-length "${QUALITY_MAX_PATH_LENGTH}"
  --quality-max-length-ratio "${QUALITY_MAX_LENGTH_RATIO}"
  --quality-max-local-turn-deg "${QUALITY_MAX_LOCAL_TURN_DEG}"
  --quality-max-arc-turn-deg "${QUALITY_MAX_ARC_TURN_DEG}"
  --quality-arc-turn-spacing "${QUALITY_ARC_TURN_SPACING}"
  --quality-arc-turn-half-window-m "${QUALITY_ARC_TURN_HALF_WINDOW_M}"
  --quality-max-self-intersections "${QUALITY_MAX_SELF_INTERSECTIONS}"
  --quality-sample-stride "${QUALITY_SAMPLE_STRIDE}"
  --quality-self-intersection-max-points "${QUALITY_SELF_INTERSECTION_MAX_POINTS}"
  --master-timeout "${MASTER_TIMEOUT}"
  --startup-timeout "${STARTUP_TIMEOUT}"
  --map-timeout "${MAP_TIMEOUT}"
  --trajectory-timeout "${TRAJECTORY_TIMEOUT}"
  --case-settle-sec "${CASE_SETTLE_SEC}"
  --cleanup-sec "${CLEANUP_SEC}"
)

if [[ -n "${S_GUIDE_PATH}" ]]; then
  cmd+=("--s-guide-path=${S_GUIDE_PATH}")
fi

echo "[run_collect_demo] 即将执行："
printf '  %q' "${cmd[@]}"
echo

"${cmd[@]}"
