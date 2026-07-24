#!/usr/bin/env zsh
set -e
set -u
set -o pipefail

# ============================================================
# AirGrasp MINCO-IRL: onboard 最终 d1/d2 规划并执行
# 使用方式：
#   1. 先启动定位链路，确认 /drone0/odom 正常。
#   2. 单独启动 Single-Drone-Planner/scripts/onboard/px4ctrl.sh。
#   3. 再运行本脚本。
#   4. RViz 确认轨迹后，发布 /irl/execute_enable=true 才会执行。
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

OPTIMIZER_RUN_DIR="${PKG_ROOT}/results/runs/irl_fd_manual_20260724_161550"  # 要执行的优化结果目录
RESULTS_DIR="${PKG_ROOT}/results/onboard_execute"       # onboard 执行记录输出目录
RUN_NAME="final_d_execute_onboard_$(date +%Y%m%d_%H%M%S)"  # 本次执行记录目录名

D_SOURCE="final"                                        # final 用最终 d；best 用历史最优 d
D1_OVERRIDE=""                                          # 留空则从优化结果读取
D2_OVERRIDE=""                                          # 留空则从优化结果读取

SCENE_CONFIG="${PKG_ROOT}/config/irl_221201_scene.yaml"  # IRL 标定场景
ONBOARD_SCENE_LAUNCH="${PKG_ROOT}/launch/irl_221201_onboard_scene.launch"  # 只发布 IRL 场景/PCD 可视化
PLANNING_LAUNCH="${PKG_ROOT}/launch/two_pillar_planning.launch"  # IRL planner

# ----------------------------
# 实机话题
# ----------------------------

ODOM_TOPIC="/drone0/odom"                               # FAST-LIO/EKF 输出定位
JOINT_STATE_TOPIC="/joint_state_est"                    # 真实机械臂关节反馈
PLANNER_LIVE_CMD_TOPIC="/irl/planner_live_position_cmd" # planner 规划阶段输出，不直接给 px4ctrl
EXECUTOR_CMD_TOPIC="/position_cmd"                      # px4ctrl 真正执行输入
SELECTED_TRAJ_TOPIC="/irl/selected_position_command_trajectory"  # replay 选中的轨迹
EXECUTE_ENABLE_TOPIC="/irl/execute_enable"              # 执行确认开关

# ----------------------------
# planner / guide 参数
# 留空表示继承 optimizer run 里的配置
# ----------------------------

SPHERE_RADIUS="0.25"                                    # UAM 球半径
OBSTACLES_INFLATION="0.40"                              # 地图障碍膨胀
SAFETY_ZONE_BUFFER="0.0"                                # 安全区可视化附加 buffer

RHO_ASTAR_WAYPOINT="100000"                             # stage1 guide warm-start 权重
RHO_ASTAR_WAYPOINT_STAGE2="0.0"                         # stage2 不再被 guide 拉住
ENABLE_TAIL_CONSTRAINT="false"                          # 不强行追加末端 tail
MINCO_WARM_START_CACHE="true"                           # 缓存第一条 warm-start
MINCO_WARM_START_CACHE_FORCE_REUSE="true"                # 后续强制复用 warm-start
IRL_COLLISION_WEIGHT_SCALE="5.0"                        # d1/d2 避障权重
IRL_OBS_OPTIMIZATION_MARGIN="0.02"                      # J_obs 优化余量

S_GUIDE_ENABLE="true"                                   # 启用 S-guide
S_GUIDE_BYPASS_ASTAR="true"                             # 不走 A*
S_GUIDE_MODE="fixed"                                    # 使用场景 reference_path
S_GUIDE_PATH=""                                         # 留空则用 scene_config/reference_path
S_GUIDE_ENDPOINT_TOLERANCE="0.45"                       # guide 起终点容差

# ----------------------------
# 轨迹选择 / 执行安全门
# ----------------------------

TRAJECTORY_SELECTION="best_quality"                     # onboard 推荐选质量最好的候选轨迹
TRAJECTORY_COLLECTION_SEC="2.0"                         # 收集候选轨迹时间
QUALITY_CLEARANCE_SOURCE="learned_d"                    # 用学到的 d 做质量 gate

AUTO_EXECUTE="false"                                    # true 会规划成功后自动执行；默认必须手动确认
REEXECUTE_SAME_ID="false"                               # 防止同一轨迹重复执行
START_DELAY_SEC="0.0"                                   # 确认执行后的延迟
HOLD_FINAL_SEC="2.0"                                    # 末端保持时间
HOLD_HZ="30.0"                                          # 末端保持发布频率

# ----------------------------
# 启动开关
# ----------------------------

START_ONBOARD_CONTROL_STACK="false"                     # 默认不启动 px4ctrl 控制链路；请单独运行 Single-Drone-Planner/scripts/onboard/px4ctrl.sh
START_ONBOARD_SCENE="true"                              # 启动 IRL 场景点云与可视化
START_EXECUTOR="true"                                   # 启动 selected trajectory executor
WAIT_FOR_TOPICS="true"                                  # 规划前检查 odom/joint 话题
REQUIRE_PX4CTRL_SUBSCRIBER="true"                       # 执行前要求 /position_cmd 已被 /px4ctrl 订阅
PX4CTRL_NODE_NAME="/px4ctrl"                            # px4ctrl 节点名

# ----------------------------
# 可视化 / 日志
# ----------------------------

USE_MINCO_FINAL_TRAJECTORY_VISUALIZER="true"            # 显示 replay 最终 MINCO 轨迹
SHOW_MINCO_REFERENCE_TRAJECTORY="false"                 # onboard 默认不显示参考轨迹，避免误判
MINCO_FINAL_LINE_WIDTH="0.12"                           # 最终轨迹线宽
MINCO_REFERENCE_LINE_WIDTH="0.17"                       # 参考轨迹线宽
STREAM_PLANNING_DEBUG="false"                           # planner debug
STREAM_PLANNING_DEBUG_STRIDE="20"                       # debug 抽样间隔
TEE_STDOUT="false"                                      # true 同步打印子进程日志

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
START_TOLERANCE="0.12"                                  # 当前 odom 距场景 start 的允许误差
TOPIC_TIMEOUT_SEC="20"                                  # 等待 odom/joint 话题时间

# ----------------------------
# 环境初始化
# ----------------------------

if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
elif command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.zsh hook)"
else
  echo "[run_final_execute_onboard] 找不到 conda，请检查 Conda 安装路径。"
  exit 1
fi

conda activate "${CONDA_ENV}"

cd "${WS_ROOT}"
source devel/setup.zsh

bg_pids=()

cleanup() {
  echo "[run_final_execute_onboard] 清理后台节点..."
  for pid in "${bg_pids[@]}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"
  shift
  echo "[run_final_execute_onboard] 启动 ${name}: $*"
  "$@" &
  bg_pids+=("$!")
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local start_sec="${SECONDS}"
  echo "[run_final_execute_onboard] 等待话题 ${topic} ..."
  while (( SECONDS - start_sec < timeout_sec )); do
    if rostopic list 2>/dev/null | grep -qx "${topic}"; then
      echo "[run_final_execute_onboard] 话题已存在: ${topic}"
      return 0
    fi
    sleep 0.5
  done
  echo "[run_final_execute_onboard] ERROR: 等待话题超时: ${topic}"
  return 1
}

wait_for_topic_subscriber() {
  local topic="$1"
  local subscriber="$2"
  local timeout_sec="$3"
  local start_sec="${SECONDS}"
  echo "[run_final_execute_onboard] 等待 ${subscriber} 订阅 ${topic} ..."
  while (( SECONDS - start_sec < timeout_sec )); do
    if rostopic info "${topic}" 2>/dev/null | grep -q "${subscriber}"; then
      echo "[run_final_execute_onboard] 已确认 ${subscriber} 订阅 ${topic}"
      return 0
    fi
    sleep 0.5
  done
  echo "[run_final_execute_onboard] ERROR: ${topic} 没有检测到 ${subscriber} 订阅。"
  echo "[run_final_execute_onboard] 请先启动："
  echo "  zsh ${WS_ROOT}/src/airgrasp_planning/Single-Drone-Planner/scripts/onboard/px4ctrl.sh"
  return 1
}

# ----------------------------
# 启动实机控制链路
# ----------------------------

if [[ "${START_ONBOARD_CONTROL_STACK}" == "true" ]]; then
  start_bg "key2UAM" roslaunch key_to_camPos key2UAM.launch
  sleep 0.5
  start_bg "pcl_sync_world" roslaunch bridge_node pcl_sync_world.launch
  sleep 1.0
  start_bg "airgrasp_urdf_real" roslaunch airgrasp_urdf display_real.launch
  sleep 0.5
  start_bg "px4ctrl_real" roslaunch px4ctrl run_ctrl_real.launch
  sleep 0.5
fi

if [[ "${START_ONBOARD_SCENE}" == "true" ]]; then
  start_bg "irl_onboard_scene" roslaunch "${ONBOARD_SCENE_LAUNCH}" \
    scene_config:="${SCENE_CONFIG}" \
    publish_point_clouds:=true \
    sphere_radius:="${SPHERE_RADIUS}" \
    initial_d1:=0.0 \
    initial_d2:=0.0 \
    safety_zone_buffer:="${SAFETY_ZONE_BUFFER}"
  sleep 1.0
fi

if [[ "${START_EXECUTOR}" == "true" ]]; then
  start_bg "selected_trajectory_executor" rosrun airgrasp_minco_irl selected_trajectory_executor.py \
    _trajectory_topic:="${SELECTED_TRAJ_TOPIC}" \
    _cmd_topic:="${EXECUTOR_CMD_TOPIC}" \
    _execute_enable_topic:="${EXECUTE_ENABLE_TOPIC}" \
    _auto_execute:="${AUTO_EXECUTE}" \
    _reexecute_same_id:="${REEXECUTE_SAME_ID}" \
    _start_delay_sec:="${START_DELAY_SEC}" \
    _hold_final_sec:="${HOLD_FINAL_SEC}" \
    _hold_hz:="${HOLD_HZ}"
  sleep 0.5
fi

if [[ "${WAIT_FOR_TOPICS}" == "true" ]]; then
  wait_for_topic "${ODOM_TOPIC}" "${TOPIC_TIMEOUT_SEC}"
  wait_for_topic "${JOINT_STATE_TOPIC}" "${TOPIC_TIMEOUT_SEC}"
fi

if [[ "${REQUIRE_PX4CTRL_SUBSCRIBER}" == "true" ]]; then
  wait_for_topic_subscriber "${EXECUTOR_CMD_TOPIC}" "${PX4CTRL_NODE_NAME}" "${TOPIC_TIMEOUT_SEC}"
fi

# ----------------------------
# 规划最终 d1/d2
# ----------------------------

cmd=(
  rosrun airgrasp_minco_irl run_final_d_replay.py
  --optimizer-run-dir "${OPTIMIZER_RUN_DIR}"
  --results-dir "${RESULTS_DIR}"
  --run-name "${RUN_NAME}"
  --d-source "${D_SOURCE}"
  --replay-mode same_process
  --same-process-trials 1
  --fresh-trials 0
  --sample-count 200
  --yaw-weight 0.0
  --single-shot-script "${PKG_ROOT}/scripts/run_single_shot_planning_eval.py"
  --scene-config "${SCENE_CONFIG}"
  --planning-launch "${PLANNING_LAUNCH}"
  --sphere-radius "${SPHERE_RADIUS}"
  --obstacles-inflation "${OBSTACLES_INFLATION}"
  --safety-zone-buffer "${SAFETY_ZONE_BUFFER}"
  --rho-astar-waypoint "${RHO_ASTAR_WAYPOINT}"
  --rho-astar-waypoint-stage2 "${RHO_ASTAR_WAYPOINT_STAGE2}"
  --enable-tail-constraint "${ENABLE_TAIL_CONSTRAINT}"
  --minco-warm-start-cache "${MINCO_WARM_START_CACHE}"
  --minco-warm-start-cache-force-reuse "${MINCO_WARM_START_CACHE_FORCE_REUSE}"
  --irl-collision-weight-scale "${IRL_COLLISION_WEIGHT_SCALE}"
  --irl-obs-optimization-margin "${IRL_OBS_OPTIMIZATION_MARGIN}"
  --odom-source external
  --odom-topic "${ODOM_TOPIC}"
  --joint-state-topic "${JOINT_STATE_TOPIC}"
  --position-cmd-topic "${PLANNER_LIVE_CMD_TOPIC}"
  --skip-world-launch true
  --require-start-near true
  --start-tolerance "${START_TOLERANCE}"
  --s-guide-enable "${S_GUIDE_ENABLE}"
  --s-guide-bypass-astar "${S_GUIDE_BYPASS_ASTAR}"
  --s-guide-mode "${S_GUIDE_MODE}"
  --s-guide-endpoint-tolerance "${S_GUIDE_ENDPOINT_TOLERANCE}"
  --trajectory-selection "${TRAJECTORY_SELECTION}"
  --trajectory-collection-sec "${TRAJECTORY_COLLECTION_SEC}"
  --quality-clearance-source "${QUALITY_CLEARANCE_SOURCE}"
  --use-minco-final-trajectory-visualizer "${USE_MINCO_FINAL_TRAJECTORY_VISUALIZER}"
  --show-minco-reference-trajectory "${SHOW_MINCO_REFERENCE_TRAJECTORY}"
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
  --fixed-odom-hz 50.0
)

if [[ -n "${D1_OVERRIDE}" ]]; then cmd+=("--d1" "${D1_OVERRIDE}"); fi
if [[ -n "${D2_OVERRIDE}" ]]; then cmd+=("--d2" "${D2_OVERRIDE}"); fi
if [[ -n "${S_GUIDE_PATH}" ]]; then cmd+=("--s-guide-path=${S_GUIDE_PATH}"); fi

echo "[run_final_execute_onboard] 即将规划最终 d1/d2："
printf '  %q' "${cmd[@]}"
echo

"${cmd[@]}"

echo
echo "[run_final_execute_onboard] 规划完成，轨迹已发布到 ${SELECTED_TRAJ_TOPIC}。"
if [[ "${AUTO_EXECUTE}" == "true" ]]; then
  echo "[run_final_execute_onboard] AUTO_EXECUTE=true，发送执行确认。"
  rostopic pub -1 "${EXECUTE_ENABLE_TOPIC}" std_msgs/Bool "data: true"
else
  echo "[run_final_execute_onboard] RViz 确认无误后，手动执行："
  echo "  rostopic pub -1 ${EXECUTE_ENABLE_TOPIC} std_msgs/Bool 'data: true'"
fi
echo "[run_final_execute_onboard] 按 Ctrl-C 结束并清理后台节点。"

while true; do
  sleep 1
done
