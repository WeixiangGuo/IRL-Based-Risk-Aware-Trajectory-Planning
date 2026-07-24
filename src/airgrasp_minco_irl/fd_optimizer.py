#!/usr/bin/python3

import argparse
import csv
import datetime as _dt
import glob
import importlib.util
import json
import math
import os
import re
import signal
import shutil
import subprocess
import sys
import time

from airgrasp_minco_irl.adaptive_guide import (
    DEFAULT_CLEARANCE_BUFFER,
    DEFAULT_MAX_CLEARANCE,
    DEFAULT_MIN_CLEARANCE,
    build_adaptive_s_guide_from_config,
    load_scene,
    parse_guide_spec,
    write_guide_artifacts,
)


DEFAULT_INIT_D1 = 0.10
DEFAULT_INIT_D2 = 0.05
DEFAULT_SPHERE_RADIUS = 0.25
DEFAULT_MAX_D = 0.80
DEFAULT_RHO_ASTAR_WAYPOINT = 100000.0
DEFAULT_RHO_ASTAR_WAYPOINT_STAGE2 = 0.0
DEFAULT_RHO_ASTAR_WAYPOINT_SCHEDULE = ""
DEFAULT_MINCO_WARM_START_CACHE = True
DEFAULT_MINCO_WARM_START_CACHE_FORCE_REUSE = True
_NUM_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_FINAL_MARGIN_RE = re.compile(
    r"stage=final.*?"
    r"d=\(\s*({num})\s*,\s*({num})\s*\).*?"
    r"clearance=\(\s*({num})\s*,\s*({num})\s*\).*?"
    r"margin=\(\s*({num})\s*,\s*({num})\s*\).*?"
    r"status=([A-Za-z0-9_]+)".format(num=_NUM_RE)
)
_PLANNING_MARGIN_CACHE = {}


def ensure_rospy_node(name):
    import rospy
    if not rospy.core.is_initialized():
        rospy.init_node(name, anonymous=True, disable_signals=True)
    return rospy


def clear_marker_array_topic(topic, label):
    import rospy
    from visualization_msgs.msg import Marker, MarkerArray

    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.2)
    marker = Marker()
    marker.action = Marker.DELETEALL
    pub.publish(MarkerArray(markers=[marker]))
    print("[fd_irl] cleared {} markers on {}".format(label, topic), flush=True)


def parse_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("Expected a boolean value, got '{}'.".format(value))


def now_tag():
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def package_root():
    env_root = os.environ.get("AIRGRASP_MINCO_IRL_ROOT")
    candidates = []
    if env_root:
        candidates.append(env_root)

    try:
        root = subprocess.check_output(
            ["rospack", "find", "airgrasp_minco_irl"],
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()
        if root:
            candidates.append(root)
    except Exception:
        pass

    candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.exists(os.path.join(candidate, "config", "two_pillar_scene.yaml")):
            return candidate

    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def latest_demo_csv(results_dir):
    pattern = os.path.join(os.path.abspath(results_dir), "**", "demo_trajectory.csv")
    candidates = glob.glob(pattern, recursive=True)
    if not candidates:
        return None
    return max(candidates, key=lambda path: os.path.getmtime(path))


def run_logged_command(cmd, stdout_log, tee_stdout=True):
    with open(stdout_log, "w") as log_file:
        if not tee_stdout:
            return subprocess.run(cmd, stdout=log_file, stderr=subprocess.STDOUT)

        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
        )
        try:
            for line in proc.stdout:
                log_file.write(line)
                log_file.flush()
                sys.stdout.write(line)
                sys.stdout.flush()
        finally:
            if proc.stdout is not None:
                proc.stdout.close()
        return subprocess.CompletedProcess(cmd, proc.wait())


def read_csv_rows(path):
    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def coerce_value(value):
    if value is None:
        return None
    text = str(value)
    if text == "":
        return text
    try:
        number = float(text)
    except ValueError:
        return value
    if number.is_integer():
        return int(number)
    return number


def coerce_row(row):
    return {key: coerce_value(value) for key, value in row.items()}


def finite_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def first_finite(*values):
    for value in values:
        number = finite_float(value)
        if number is not None:
            return number
    return None


def parse_planning_final_margin_entries(log_path):
    log_path = os.path.abspath(log_path)
    if log_path in _PLANNING_MARGIN_CACHE:
        return _PLANNING_MARGIN_CACHE[log_path]
    entries = []
    if not os.path.exists(log_path):
        _PLANNING_MARGIN_CACHE[log_path] = entries
        return entries
    with open(log_path, "r", errors="replace") as f:
        for line_no, line in enumerate(f, start=1):
            match = _FINAL_MARGIN_RE.search(line)
            if not match:
                continue
            entries.append({
                "line": line_no,
                "d1": float(match.group(1)),
                "d2": float(match.group(2)),
                "clearance_1": float(match.group(3)),
                "clearance_2": float(match.group(4)),
                "margin_1": float(match.group(5)),
                "margin_2": float(match.group(6)),
                "status": match.group(7),
                "source": os.path.abspath(log_path),
            })
    _PLANNING_MARGIN_CACHE[log_path] = entries
    return entries


def invalidate_planning_margin_cache(*paths):
    for path in paths:
        if not path:
            continue
        _PLANNING_MARGIN_CACHE.pop(os.path.abspath(path), None)


def find_planning_final_margin(eval_run_dir, d1, d2, tol=5e-4):
    candidate_logs = [
        os.path.join(eval_run_dir, "planning.log"),
        os.path.join(os.path.dirname(eval_run_dir), "planning.log"),
    ]
    for log_path in candidate_logs:
        entries = parse_planning_final_margin_entries(log_path)
        matches = [
            entry for entry in entries
            if abs(entry["d1"] - d1) <= tol and abs(entry["d2"] - d2) <= tol
        ]
        if matches:
            return matches[-1]
    return None


def summarize_feasibility(args, status, summary_typed, eval_run_dir, d1, d2):
    margin_1 = first_finite(
        summary_typed.get("candidate_min_margin_1"),
        summary_typed.get("planned_min_margin_1"),
        summary_typed.get("executed_min_margin_1"),
    )
    margin_2 = first_finite(
        summary_typed.get("candidate_min_margin_2"),
        summary_typed.get("planned_min_margin_2"),
        summary_typed.get("executed_min_margin_2"),
    )
    clearance_1 = first_finite(
        summary_typed.get("candidate_min_clearance_1"),
        summary_typed.get("planned_min_clearance_1"),
        summary_typed.get("executed_min_clearance_1"),
    )
    clearance_2 = first_finite(
        summary_typed.get("candidate_min_clearance_2"),
        summary_typed.get("planned_min_clearance_2"),
        summary_typed.get("executed_min_clearance_2"),
    )
    source = "summary" if margin_1 is not None or margin_2 is not None else ""

    if margin_1 is None or margin_2 is None:
        entry = find_planning_final_margin(eval_run_dir, d1, d2)
        if entry is not None:
            margin_1 = entry["margin_1"] if margin_1 is None else margin_1
            margin_2 = entry["margin_2"] if margin_2 is None else margin_2
            clearance_1 = entry["clearance_1"] if clearance_1 is None else clearance_1
            clearance_2 = entry["clearance_2"] if clearance_2 is None else clearance_2
            source = "planning_log_final"

    violation_1 = max(0.0, -margin_1) if margin_1 is not None else None
    violation_2 = max(0.0, -margin_2) if margin_2 is not None else None
    violation_sum = 0.0
    available = False
    for violation in (violation_1, violation_2):
        if violation is not None:
            available = True
            violation_sum += violation * violation

    return {
        "available": available,
        "source": source,
        "min_clearance_1": clearance_1,
        "min_clearance_2": clearance_2,
        "min_margin_1": margin_1,
        "min_margin_2": margin_2,
        "violation_1": violation_1,
        "violation_2": violation_2,
        "loss": float(args.feasibility_loss_weight) * violation_sum if available else None,
    }


def compose_outer_loss(args, status, trajectory_losses, feasibility, completed_wp, waypoint_count, has_points):
    imitation_loss = finite_float(trajectory_losses.get("loss"))
    feasibility_loss = finite_float(feasibility.get("loss"))
    missing = max(0, waypoint_count - completed_wp)

    if status == "success" and has_points and imitation_loss is not None:
        return {
            "loss": imitation_loss + (feasibility_loss or 0.0),
            "loss_mode": "imitation_plus_feasibility",
            "imitation_loss": imitation_loss,
            "feasibility_loss": feasibility_loss or 0.0,
            "failure_status_penalty_applied": 0.0,
        }

    if status == "failed_quality_gate" and has_points and imitation_loss is not None:
        penalty = float(args.failure_status_penalty)
        return {
            "loss": imitation_loss + (feasibility_loss or 0.0) + penalty,
            "loss_mode": "quality_failed_imitation_plus_feasibility",
            "imitation_loss": imitation_loss,
            "feasibility_loss": feasibility_loss or 0.0,
            "failure_status_penalty_applied": penalty,
        }

    if args.failure_loss_mode == "feasibility_boundary" and feasibility.get("available"):
        penalty = float(args.failure_status_penalty)
        return {
            "loss": (feasibility_loss or 0.0) + penalty,
            "loss_mode": "feasibility_boundary",
            "imitation_loss": "",
            "feasibility_loss": feasibility_loss or 0.0,
            "failure_status_penalty_applied": penalty,
        }

    loss = float(args.failure_loss_penalty) + missing * float(args.missing_goal_penalty)
    return {
        "loss": loss,
        "loss_mode": "constant_failure",
        "imitation_loss": "",
        "feasibility_loss": "",
        "failure_status_penalty_applied": float(args.failure_loss_penalty),
    }


def read_trajectory(path):
    points = []
    for row in read_csv_rows(path):
        if not {"x", "y", "z"}.issubset(row.keys()):
            raise RuntimeError("Trajectory CSV must include x,y,z columns: {}".format(path))
        points.append({
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row["z"]),
            "yaw": float(row.get("yaw", 0.0) or 0.0),
        })
    if len(points) < 2:
        raise RuntimeError("Trajectory needs at least 2 points: {}".format(path))
    return remove_duplicate_positions(points)


def remove_duplicate_positions(points, eps=1e-7):
    clean = []
    last = None
    for point in points:
        if last is None or point_distance(last, point) > eps:
            clean.append(point)
            last = point
    if len(clean) == 1 and len(points) > 1:
        clean.append(points[-1])
    return clean


def write_trajectory(path, points):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["point_index", "x", "y", "z", "yaw"])
        writer.writeheader()
        for idx, point in enumerate(points):
            writer.writerow({
                "point_index": idx,
                "x": "{:.9f}".format(point["x"]),
                "y": "{:.9f}".format(point["y"]),
                "z": "{:.9f}".format(point["z"]),
                "yaw": "{:.9f}".format(point.get("yaw", 0.0)),
            })


def read_planned_trajectories(path):
    trajectories = []
    current_key = None
    current = None
    for row in read_csv_rows(path):
        if not {"trajectory_index", "x", "y", "z"}.issubset(row.keys()):
            raise RuntimeError("Planned trajectory CSV has unexpected columns: {}".format(path))
        key = row["trajectory_index"]
        if key != current_key:
            current = {
                "trajectory_index": int(float(key)),
                "trajectory_id": int(float(row.get("trajectory_id", 0) or 0)),
                "trajectory_stamp": float(row.get("trajectory_stamp", 0.0) or 0.0),
                "points": [],
            }
            trajectories.append(current)
            current_key = key
        current["points"].append({
            "x": float(row["x"]),
            "y": float(row["y"]),
            "z": float(row["z"]),
            "yaw": float(row.get("yaw", 0.0) or 0.0),
        })
    return [traj for traj in trajectories if len(traj["points"]) >= 2]


def target_key(point, decimals=3):
    return (
        round(point["x"], decimals),
        round(point["y"], decimals),
        round(point["z"], decimals),
    )


def concatenate_trajectories(trajectories):
    points = []
    for traj in trajectories:
        for point in traj["points"]:
            if points and point_distance(points[-1], point) < 1e-7:
                continue
            points.append(point)
    return remove_duplicate_positions(points)


def read_planned_candidate(path, selection):
    trajectories = read_planned_trajectories(path)
    if not trajectories:
        raise RuntimeError("No planned trajectories found in {}".format(path))
    if selection == "all":
        return concatenate_trajectories(trajectories), len(trajectories)

    selected_by_goal = {}
    goal_order = []
    for traj in trajectories:
        key = target_key(traj["points"][-1])
        if key not in selected_by_goal:
            goal_order.append(key)
            selected_by_goal[key] = traj
        elif selection == "last_by_goal":
            selected_by_goal[key] = traj

    selected = [selected_by_goal[key] for key in goal_order]
    return concatenate_trajectories(selected), len(selected)


def load_candidate_points(args, case_dir, eval_run_dir, summary=None):
    if args.eval_mode == "single_shot":
        summary = summary or {}
        trajectory_csv = summary.get("candidate_trajectory_csv") or os.path.join(case_dir, "candidate_planned_path.csv")
        source_csv = summary.get("source_trajectory_csv") or os.path.join(case_dir, "planned_trajectories.csv")
        selected_count = int(float(summary.get("planned_traj_count", 1) or 0))
        return (
            read_trajectory(trajectory_csv),
            os.path.abspath(trajectory_csv),
            os.path.abspath(source_csv),
            selected_count,
        )

    if args.candidate_source == "executed":
        trajectory_csv = os.path.join(case_dir, "executed_path.csv")
        return read_trajectory(trajectory_csv), os.path.abspath(trajectory_csv), trajectory_csv, 0

    source_csv = os.path.join(case_dir, "planned_trajectories.csv")
    points, selected_count = read_planned_candidate(source_csv, args.planned_selection)
    candidate_csv = os.path.join(eval_run_dir, "candidate_planned_path.csv")
    write_trajectory(candidate_csv, points)
    return points, os.path.abspath(candidate_csv), os.path.abspath(source_csv), selected_count


def point_distance(a, b):
    return math.sqrt(
        (a["x"] - b["x"]) * (a["x"] - b["x"]) +
        (a["y"] - b["y"]) * (a["y"] - b["y"]) +
        (a["z"] - b["z"]) * (a["z"] - b["z"])
    )


def path_length(points):
    total = 0.0
    for idx in range(1, len(points)):
        total += point_distance(points[idx - 1], points[idx])
    return total


def unwrap_yaws(points):
    if not points:
        return []
    values = [points[0]["yaw"]]
    for point in points[1:]:
        yaw = point["yaw"]
        prev = values[-1]
        while yaw - prev > math.pi:
            yaw -= 2.0 * math.pi
        while yaw - prev < -math.pi:
            yaw += 2.0 * math.pi
        values.append(yaw)
    return values


def resample_by_path_length(points, sample_count):
    if sample_count <= 1:
        raise ValueError("sample_count must be greater than 1.")
    points = remove_duplicate_positions(points)
    total = path_length(points)
    if total <= 1e-9:
        return [dict(points[0]) for _ in range(sample_count)]

    yaws = unwrap_yaws(points)
    cumulative = [0.0]
    for idx in range(1, len(points)):
        cumulative.append(cumulative[-1] + point_distance(points[idx - 1], points[idx]))

    sampled = []
    seg_idx = 1
    for sample_idx in range(sample_count):
        target_s = total * float(sample_idx) / float(sample_count - 1)
        while seg_idx < len(cumulative) - 1 and cumulative[seg_idx] < target_s:
            seg_idx += 1
        prev_s = cumulative[seg_idx - 1]
        next_s = cumulative[seg_idx]
        if next_s <= prev_s:
            alpha = 0.0
        else:
            alpha = (target_s - prev_s) / (next_s - prev_s)
        prev_p = points[seg_idx - 1]
        next_p = points[seg_idx]
        sampled.append({
            "x": prev_p["x"] + alpha * (next_p["x"] - prev_p["x"]),
            "y": prev_p["y"] + alpha * (next_p["y"] - prev_p["y"]),
            "z": prev_p["z"] + alpha * (next_p["z"] - prev_p["z"]),
            "yaw": yaws[seg_idx - 1] + alpha * (yaws[seg_idx] - yaws[seg_idx - 1]),
        })
    return sampled


def angle_diff(a, b):
    diff = a - b
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def trajectory_loss(demo_samples, candidate_points, sample_count, yaw_weight):
    candidate_samples = resample_by_path_length(candidate_points, sample_count)
    pos_loss = 0.0
    yaw_loss = 0.0
    for demo, candidate in zip(demo_samples, candidate_samples):
        dx = demo["x"] - candidate["x"]
        dy = demo["y"] - candidate["y"]
        dz = demo["z"] - candidate["z"]
        pos_loss += dx * dx + dy * dy + dz * dz
        dyaw = angle_diff(demo["yaw"], candidate["yaw"])
        yaw_loss += dyaw * dyaw
    pos_loss /= float(sample_count)
    yaw_loss /= float(sample_count)
    return {
        "loss": pos_loss + yaw_weight * yaw_loss,
        "position_loss": pos_loss,
        "yaw_loss": yaw_loss,
    }


def cylinder_surface_distance(point, center_xy, radius, z_min, z_max):
    radial_gap = math.hypot(point["x"] - center_xy[0], point["y"] - center_xy[1]) - radius
    if z_min <= point["z"] <= z_max:
        return radial_gap
    z_gap = z_min - point["z"] if point["z"] < z_min else point["z"] - z_max
    if radial_gap > 0.0:
        return math.sqrt(radial_gap * radial_gap + z_gap * z_gap)
    return z_gap


def trajectory_clearance_stats(points, scene, sphere_radius):
    min_c1 = float("inf")
    min_c2 = float("inf")
    for point in points:
        c1 = cylinder_surface_distance(
            point,
            scene["pillar1_center_xy"],
            scene["pillar1_radius"],
            scene["pillar_z_min"],
            scene["pillar_z_max"],
        ) - sphere_radius
        c2 = cylinder_surface_distance(
            point,
            scene["pillar2_center_xy"],
            scene["pillar2_radius"],
            scene["pillar_z_min"],
            scene["pillar_z_max"],
        ) - sphere_radius
        min_c1 = min(min_c1, c1)
        min_c2 = min(min_c2, c2)
    return {
        "min_clearance_1": min_c1,
        "min_clearance_2": min_c2,
    }


def demo_clearance_floor(args, iteration):
    if not args.s_guide_demo_floor_enable:
        return None, None, None
    margin = float(args.s_guide_demo_floor_start_margin)
    return (
        float(args.s_guide_demo_clearance_1) + margin,
        float(args.s_guide_demo_clearance_2) + margin,
        margin,
    )


def materialize_initial_s_guide(args, run_dir):
    if args.s_guide_mode != "adaptive":
        args.initial_s_guide_info = None
        return None

    init_d1 = bounded(args.init_d1, args.min_d, args.max_d)
    init_d2 = bounded(args.init_d2, args.min_d, args.max_d)
    clearance_floor_1, clearance_floor_2, demo_floor_margin = demo_clearance_floor(args, 0)
    path, metadata = build_adaptive_s_guide_from_config(
        args.scene_config,
        init_d1,
        init_d2,
        sphere_radius=args.sphere_radius,
        clearance_buffer=args.s_guide_clearance_buffer,
        min_clearance=args.s_guide_min_clearance,
        max_clearance=args.s_guide_max_clearance,
        clearance_floor_1=clearance_floor_1,
        clearance_floor_2=clearance_floor_2,
    )
    guide_dir = os.path.join(run_dir, "initial_s_guide")
    csv_path, json_path = write_guide_artifacts(
        guide_dir, path, metadata, prefix="initial_s_guide"
    )
    metadata.update({
        "s_guide_requested_mode": "adaptive",
        "s_guide_mode": "fixed_initial",
        "s_guide_path": metadata["path_spec"],
        "s_guide_csv": os.path.abspath(csv_path),
        "s_guide_json": os.path.abspath(json_path),
        "s_guide_demo_clearance_1": args.s_guide_demo_clearance_1,
        "s_guide_demo_clearance_2": args.s_guide_demo_clearance_2,
        "s_guide_demo_floor_margin": demo_floor_margin,
    })

    args.initial_s_guide_info = metadata
    args.s_guide_path = metadata["path_spec"]
    args.s_guide_mode = "fixed"
    print(
        "[fd_irl] materialized fixed initial S-guide from init d=({:.4f}, {:.4f}), "
        "points={}, clearance=({:.4f}, {:.4f})".format(
            init_d1,
            init_d2,
            metadata["guide_point_count"],
            metadata["guide_min_clearance_1"],
            metadata["guide_min_clearance_2"],
        ),
        flush=True,
    )
    return metadata


def guide_points_for_visualization(args, scene):
    if not args.s_guide_enable:
        return []
    if args.s_guide_path:
        return parse_guide_spec(args.s_guide_path)
    return [
        {"x": point[0], "y": point[1], "z": point[2]}
        for point in scene.get("reference_path", [])
    ]


class ManagedProcess:
    def __init__(self, args, log_path):
        self.log_file = open(log_path, "w")
        self.proc = subprocess.Popen(
            list(args),
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

    def terminate(self, timeout=8.0):
        if self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except OSError:
                pass
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                except OSError:
                    pass
                try:
                    self.proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                    except OSError:
                        pass
                    self.proc.wait(timeout=2.0)
        self.log_file.close()


def master_is_online():
    try:
        import rosgraph
        rosgraph.Master("/airgrasp_fd_irl_probe").getPid()
        return True
    except Exception:
        return False


def wait_for_master(timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if master_is_online():
            return True
        time.sleep(0.2)
    return False


def ensure_ros_master(run_dir, timeout):
    if master_is_online():
        return None
    proc = ManagedProcess(["roscore"], os.path.join(run_dir, "roscore.log"))
    if not wait_for_master(timeout):
        proc.terminate()
        raise RuntimeError("ROS master did not start within {:.1f}s".format(timeout))
    return proc


class IrlVisualization:
    def __init__(
        self,
        topic,
        frame_id,
        line_stride,
        scene=None,
        sphere_radius=DEFAULT_SPHERE_RADIUS,
        minco_history_limit=300,
    ):
        import rospy
        from visualization_msgs.msg import MarkerArray

        self.rospy = rospy
        self.topic = topic
        self.frame_id = frame_id
        self.line_stride = max(1, int(line_stride))
        self.scene = dict(scene) if scene is not None else None
        self.sphere_radius = float(sphere_radius)
        self.current_d1 = None
        self.current_d2 = None
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
        self.demo_points = []
        self.guide_points = []
        self.history = []
        self.minco_history = []
        self.minco_history_limit = int(minco_history_limit)
        self.minco_sequence = 0
        self.status_text = "Waiting for optimizer."
        time.sleep(0.5)

    def set_demo(self, points):
        self.demo_points = list(points)
        self.publish()

    def set_guide(self, points):
        self.guide_points = [self._normalize_point(p) for p in points]
        self.publish()

    def set_status(self, text):
        self.status_text = text
        self.publish()

    def set_d_values(self, d1, d2):
        self.current_d1 = float(d1)
        self.current_d2 = float(d2)
        self.publish()

    def set_eval_status(self, iteration, label, d1, d2, phase, loss=None):
        loss_line = ""
        if loss is not None:
            loss_line = "\nloss: {:.6f}".format(loss)
        self.current_d1 = float(d1)
        self.current_d2 = float(d2)
        self.set_status(
            "FD IRL optimizer\n"
            "iter: {iteration}\n"
            "eval: {label}\n"
            "d1: {d1:.4f}   d2: {d2:.4f}\n"
            "phase: {phase}{loss_line}\n"
            "viz: red/orange=MINCO raw, thick=color current".format(
                iteration=iteration,
                label=label,
                d1=d1,
                d2=d2,
                phase=phase,
                loss_line=loss_line,
            )
        )

    def add_iteration(self, iteration, d1, d2, loss, points, status, extra_text=""):
        self.current_d1 = float(d1)
        self.current_d2 = float(d2)
        self.history.append({
            "iteration": iteration,
            "d1": d1,
            "d2": d2,
            "loss": loss,
            "points": list(points),
            "status": status,
        })
        self.status_text = (
            "FD IRL optimizer\n"
            "iter: {iteration}\n"
            "d1: {d1:.4f}   d2: {d2:.4f}\n"
            "loss: {loss:.6f}\n"
            "status: {status}\n"
            "viz: red/orange=MINCO raw, thick=color current{extra}"
        ).format(
            iteration=iteration,
            d1=d1,
            d2=d2,
            loss=loss,
            status=status,
            extra=("\n" + extra_text) if extra_text else "",
        )
        self.publish()

    def add_minco_trajectories(self, iteration, label, d1, d2, trajectories, selected_index=None):
        added = 0
        for traj_idx, trajectory in enumerate(trajectories or []):
            points = [
                {
                    "x": point["position"][0],
                    "y": point["position"][1],
                    "z": point["position"][2],
                }
                for point in trajectory.get("points", [])
            ]
            if len(points) < 2:
                continue
            self.minco_history.append({
                "sequence": self.minco_sequence,
                "iteration": iteration,
                "label": label,
                "d1": d1,
                "d2": d2,
                "trajectory_index": traj_idx,
                "trajectory_id": trajectory.get("trajectory_id", ""),
                "selected": selected_index is not None and traj_idx == selected_index,
                "points": points,
            })
            self.minco_sequence += 1
            added += 1
        if self.minco_history_limit > 0 and len(self.minco_history) > self.minco_history_limit:
            self.minco_history = self.minco_history[-self.minco_history_limit:]
        if added:
            self.publish()

    def publish(self):
        from visualization_msgs.msg import MarkerArray

        markers = [self._delete_all_marker()]
        marker_id = 1
        if self.guide_points:
            guide_z_offset = -0.080
            markers.append(self._line_marker(
                "fd_irl_s_guide",
                5000,
                self.guide_points,
                (0.0, 0.32, 1.00, 0.46),
                0.028,
                guide_z_offset,
                stride=1,
            ))
            for idx, point in enumerate(self.guide_points):
                is_endpoint = idx == 0 or idx == len(self.guide_points) - 1
                scale = 0.13 if is_endpoint else 0.10
                color = (0.0, 0.16, 1.00, 0.72) if is_endpoint else (0.0, 0.62, 1.00, 0.58)
                markers.append(self._endpoint_marker(
                    "fd_irl_s_guide_anchor",
                    5010 + idx,
                    point,
                    color,
                    scale,
                    guide_z_offset,
                ))

        if self.demo_points:
            markers.append(self._line_marker(
                "fd_irl_demo",
                marker_id,
                self.demo_points,
                (0.0, 0.95, 0.20, 0.95),
                0.065,
                0.035,
            ))
            marker_id += 1
            markers.append(self._endpoint_marker("fd_irl_demo_start", marker_id, self.demo_points[0],
                                                 (0.0, 0.95, 0.20, 0.95), 0.16, 0.035))
            marker_id += 1
            markers.append(self._endpoint_marker("fd_irl_demo_goal", marker_id, self.demo_points[-1],
                                                 (0.0, 0.95, 0.20, 0.95), 0.16, 0.035))
            marker_id += 1

        for display_idx, item in enumerate(self.minco_history):
            is_latest = display_idx >= max(0, len(self.minco_history) - 8)
            is_selected = bool(item.get("selected", False))
            alpha = 0.12 if not is_latest else 0.30
            width = 0.010 if not is_selected else 0.018
            color = (1.00, 0.03, 0.02, alpha) if not is_selected else (1.00, 0.22, 0.02, 0.42)
            markers.append(self._line_marker(
                "fd_irl_minco_raw_outputs",
                20000 + int(item["sequence"]),
                item["points"],
                color,
                width,
                0.045,
            ))

        for display_idx, item in enumerate(self.history):
            is_latest = display_idx == len(self.history) - 1
            alpha = 0.48 if not is_latest else 1.00
            width = 0.060 if not is_latest else 0.130
            color = self._iteration_color(display_idx, len(self.history), alpha)
            markers.append(self._line_marker(
                "fd_irl_iteration_paths",
                1000 + item["iteration"],
                item["points"],
                color,
                width,
                0.155,
            ))

        markers.extend(self._d_boundary_markers(7000))
        markers.append(self._text_marker(9000, self.status_text))
        self.pub.publish(MarkerArray(markers=markers))

    def _delete_all_marker(self):
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def _normalize_point(self, point):
        if isinstance(point, dict):
            return {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "z": float(point["z"]),
            }
        return {
            "x": float(point[0]),
            "y": float(point[1]),
            "z": float(point[2]),
        }

    def _line_marker(self, ns, marker_id, points, color, width, z_offset, stride=None):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = self.rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        selected = points[::max(1, int(stride or self.line_stride))]
        if selected and selected[-1] is not points[-1]:
            selected = selected + [points[-1]]
        marker.points = [
            Point(x=p["x"], y=p["y"], z=p["z"] + z_offset)
            for p in selected
        ]
        return marker

    def _endpoint_marker(self, ns, marker_id, point, color, scale, z_offset):
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = self.rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point["x"]
        marker.pose.position.y = point["y"]
        marker.pose.position.z = point["z"] + z_offset
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _d_boundary_markers(self, marker_id_start):
        if self.scene is None or self.current_d1 is None or self.current_d2 is None:
            return []

        z_min = float(self.scene["pillar_z_min"])
        z_max = float(self.scene["pillar_z_max"])
        start_z = float(self.scene["start_position"][2])
        goal_z = float(self.scene["goal_position"][2])
        z = min(max(0.5 * (start_z + goal_z), z_min), z_max)
        configs = [
            (
                "fd_irl_d1_boundary",
                marker_id_start,
                self.scene["pillar1_center_xy"],
                float(self.scene["pillar1_radius"]) + self.sphere_radius,
                self.current_d1,
                (0.05, 0.45, 1.00, 0.18),
                (0.02, 0.25, 1.00, 0.90),
            ),
            (
                "fd_irl_d2_boundary",
                marker_id_start + 10,
                self.scene["pillar2_center_xy"],
                float(self.scene["pillar2_radius"]) + self.sphere_radius,
                self.current_d2,
                (1.00, 0.50, 0.04, 0.18),
                (1.00, 0.25, 0.02, 0.90),
            ),
        ]
        markers = []
        for ns, marker_id, center_xy, inner_radius, d_value, fill_color, line_color in configs:
            outer_radius = inner_radius + max(0.0, float(d_value))
            markers.append(self._annulus_marker(ns, marker_id, center_xy, inner_radius, outer_radius, z, fill_color))
            markers.append(self._circle_marker(ns + "_outer", marker_id + 1, center_xy, outer_radius, z, line_color, 0.035))
            markers.append(self._circle_marker(ns + "_inner", marker_id + 2, center_xy, inner_radius, z, (0.08, 0.08, 0.08, 0.45), 0.014))
        return markers

    def _annulus_marker(self, ns, marker_id, center_xy, inner_radius, outer_radius, z, color, segments=96):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = self.rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        cx, cy = float(center_xy[0]), float(center_xy[1])
        inner_radius = max(0.0, float(inner_radius))
        outer_radius = max(inner_radius, float(outer_radius))
        for idx in range(int(segments)):
            a0 = 2.0 * math.pi * float(idx) / float(segments)
            a1 = 2.0 * math.pi * float(idx + 1) / float(segments)
            outer0 = Point(x=cx + outer_radius * math.cos(a0), y=cy + outer_radius * math.sin(a0), z=z)
            outer1 = Point(x=cx + outer_radius * math.cos(a1), y=cy + outer_radius * math.sin(a1), z=z)
            inner0 = Point(x=cx + inner_radius * math.cos(a0), y=cy + inner_radius * math.sin(a0), z=z)
            inner1 = Point(x=cx + inner_radius * math.cos(a1), y=cy + inner_radius * math.sin(a1), z=z)
            marker.points.extend([outer0, outer1, inner1, outer0, inner1, inner0])
        return marker

    def _circle_marker(self, ns, marker_id, center_xy, radius, z, color, width, segments=128):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = self.rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        cx, cy = float(center_xy[0]), float(center_xy[1])
        radius = max(0.0, float(radius))
        for idx in range(int(segments) + 1):
            angle = 2.0 * math.pi * float(idx) / float(segments)
            marker.points.append(Point(
                x=cx + radius * math.cos(angle),
                y=cy + radius * math.sin(angle),
                z=z + 0.015,
            ))
        return marker

    def _text_marker(self, marker_id, text):
        from visualization_msgs.msg import Marker

        marker = Marker()
        marker.header.stamp = self.rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = "fd_irl_status"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = -2.35
        marker.pose.position.y = -1.75
        marker.pose.position.z = 2.35
        marker.scale.z = 0.18
        marker.color.r = 0.08
        marker.color.g = 0.10
        marker.color.b = 0.12
        marker.color.a = 1.0
        marker.text = text
        return marker

    def _iteration_color(self, idx, total, alpha):
        palette = [
            (0.45, 0.45, 0.50),
            (0.12, 0.62, 1.00),
            (0.35, 0.42, 1.00),
            (0.64, 0.30, 1.00),
            (1.00, 0.46, 0.12),
            (1.00, 0.12, 0.18),
        ]
        if total <= 1:
            rgb = palette[-1]
        else:
            rgb = palette[min(idx, len(palette) - 1)]
        return rgb[0], rgb[1], rgb[2], alpha


def format_case_pair(d1, d2):
    return "{:.9f},{:.9f}".format(d1, d2)


def parse_float_schedule(value):
    values = []
    for item in str(value or "").replace(";", ",").split(","):
        text = item.strip()
        if not text:
            continue
        try:
            number = float(text)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                "Invalid --rho-astar-waypoint-schedule value '{}'.".format(text)
            ) from exc
        if number < 0.0:
            raise argparse.ArgumentTypeError("--rho-astar-waypoint-schedule values must be non-negative.")
        values.append(number)
    return values


def rho_astar_waypoint_for_iteration(args, iteration):
    schedule = getattr(args, "rho_astar_waypoint_schedule_values", None)
    if schedule:
        return schedule[min(max(iteration, 0), len(schedule) - 1)]
    return args.rho_astar_waypoint


def prepare_single_shot_guide_args(args, eval_run_dir, d1=None, d2=None, iteration=0):
    if args.s_guide_path is not None:
        guide_info = {
            "s_guide_mode": "fixed",
            "s_guide_path": args.s_guide_path,
        }
        initial_info = getattr(args, "initial_s_guide_info", None)
        if initial_info:
            guide_info.update({
                "s_guide_mode": initial_info.get("s_guide_mode", "fixed_initial"),
                "s_guide_csv": initial_info.get("s_guide_csv", ""),
                "s_guide_json": initial_info.get("s_guide_json", ""),
                "s_guide_target_clearance_1": initial_info.get("target_clearance_1", ""),
                "s_guide_target_clearance_2": initial_info.get("target_clearance_2", ""),
                "s_guide_min_clearance_1": initial_info.get("guide_min_clearance_1", ""),
                "s_guide_min_clearance_2": initial_info.get("guide_min_clearance_2", ""),
                "s_guide_clearance_floor_1": initial_info.get("clearance_floor_1", ""),
                "s_guide_clearance_floor_2": initial_info.get("clearance_floor_2", ""),
                "s_guide_demo_clearance_1": initial_info.get("s_guide_demo_clearance_1", ""),
                "s_guide_demo_clearance_2": initial_info.get("s_guide_demo_clearance_2", ""),
                "s_guide_demo_floor_margin": initial_info.get("s_guide_demo_floor_margin", ""),
            })
        return ["--s-guide-path={}".format(args.s_guide_path)], guide_info
    return [], {
        "s_guide_mode": "fixed",
        "s_guide_path": "",
    }


def build_single_shot_eval_command(args, eval_results_dir, eval_name, cases_text, eval_run_dir, d1=None, d2=None, iteration=0):
    rho_astar_waypoint = rho_astar_waypoint_for_iteration(args, iteration)
    cmd = [
        "/usr/bin/python3",
        os.path.abspath(args.single_shot_script),
        "--cases", cases_text,
        "--sphere-radius", str(args.sphere_radius),
        "--scene-config", os.path.abspath(args.scene_config),
        "--results-dir", os.path.abspath(eval_results_dir),
        "--run-name", eval_name,
        "--obstacles-inflation", str(args.obstacles_inflation),
        "--safety-zone-buffer", str(args.safety_zone_buffer),
        "--rho-astar-waypoint", str(rho_astar_waypoint),
        "--rho-astar-waypoint-stage2", str(args.rho_astar_waypoint_stage2),
        "--enable-tail-constraint", str(args.enable_tail_constraint).lower(),
        "--minco-warm-start-cache", str(args.minco_warm_start_cache).lower(),
        "--minco-warm-start-cache-force-reuse", str(args.minco_warm_start_cache_force_reuse).lower(),
        "--irl-collision-weight-scale", str(args.irl_collision_weight_scale),
        "--irl-obs-optimization-margin", str(args.irl_obs_optimization_margin),
        "--master-timeout", str(args.master_timeout),
        "--startup-timeout", str(args.startup_timeout),
        "--map-timeout", str(args.map_timeout),
        "--trajectory-timeout", str(args.trajectory_timeout),
        "--reset-settle-sec", str(args.reset_settle_sec),
        "--case-settle-sec", str(args.case_settle_sec),
        "--cleanup-sec", str(args.cleanup_sec),
        "--fixed-odom-hz", str(args.fixed_odom_hz),
        "--start-tolerance", str(args.start_tolerance),
        "--s-guide-enable", str(args.s_guide_enable).lower(),
        "--s-guide-bypass-astar", str(args.s_guide_bypass_astar).lower(),
        "--s-guide-endpoint-tolerance", str(args.s_guide_endpoint_tolerance),
        "--use-minco-final-trajectory-visualizer", "false",
        "--stream-planning-debug", str(args.stream_minco_debug).lower(),
        "--stream-planning-debug-stride", str(args.stream_minco_debug_stride),
    ]
    guide_cmd, guide_info = prepare_single_shot_guide_args(args, eval_run_dir, d1, d2, iteration)
    guide_info["rho_astar_waypoint"] = rho_astar_waypoint
    guide_info["rho_astar_waypoint_stage2"] = args.rho_astar_waypoint_stage2
    guide_info["enable_tail_constraint"] = args.enable_tail_constraint
    guide_info["minco_warm_start_cache"] = args.minco_warm_start_cache
    guide_info["minco_warm_start_cache_force_reuse"] = args.minco_warm_start_cache_force_reuse
    guide_info["irl_collision_weight_scale"] = args.irl_collision_weight_scale
    guide_info["irl_obs_optimization_margin"] = args.irl_obs_optimization_margin
    guide_info["safety_zone_buffer"] = args.safety_zone_buffer
    cmd.extend(guide_cmd)
    if args.world_launch is not None:
        cmd.extend(["--world-launch", args.world_launch])
    if args.planning_launch is not None:
        cmd.extend(["--planning-launch", args.planning_launch])
    if args.keep_eval_launch_on_failure:
        cmd.append("--keep-launch-on-failure")
    return cmd, guide_info


def build_eval_command(args, eval_results_dir, eval_name, d1, d2, iteration=0):
    return build_single_shot_eval_command(
        args,
        eval_results_dir,
        eval_name,
        format_case_pair(d1, d2),
        os.path.join(eval_results_dir, eval_name),
        d1,
        d2,
        iteration,
    )


def load_summary_rows(run_dir):
    summary_csv = os.path.join(run_dir, "summary.csv")
    return read_csv_rows(summary_csv)


def load_single_summary(run_dir):
    rows = load_summary_rows(run_dir)
    if len(rows) != 1:
        raise RuntimeError("Expected one summary row in {}".format(os.path.join(run_dir, "summary.csv")))
    return rows[0]


def failed_eval(label, d1, d2, run_dir, error, args, guide_info=None):
    guide_info = guide_info or {}
    summary_typed = {}
    feasibility = summarize_feasibility(args, "script_failed", summary_typed, run_dir, d1, d2)
    loss_info = compose_outer_loss(args, "script_failed", {}, feasibility, 0, 0, False)
    return {
        "label": label,
        "d1": d1,
        "d2": d2,
        "loss": loss_info["loss"],
        "loss_mode": loss_info["loss_mode"],
        "imitation_loss": loss_info["imitation_loss"],
        "position_loss": "",
        "yaw_loss": 0.0,
        "feasibility_loss": loss_info["feasibility_loss"],
        "feasibility_margin_available": feasibility["available"],
        "feasibility_margin_source": feasibility["source"],
        "feasibility_min_clearance_1": feasibility["min_clearance_1"],
        "feasibility_min_clearance_2": feasibility["min_clearance_2"],
        "feasibility_min_margin_1": feasibility["min_margin_1"],
        "feasibility_min_margin_2": feasibility["min_margin_2"],
        "feasibility_violation_1": feasibility["violation_1"],
        "feasibility_violation_2": feasibility["violation_2"],
        "failure_status_penalty_applied": loss_info["failure_status_penalty_applied"],
        "status": "script_failed",
        "completed_waypoints": 0,
        "waypoint_count": 0,
        "failure_count": 1,
        "failure_reasons": str(error),
        "executed_path_length": 0.0,
        "executed_min_clearance_1": "",
        "executed_min_clearance_2": "",
        "executed_min_margin_1": "",
        "executed_min_margin_2": "",
        "candidate_path_length": "",
        "candidate_min_clearance_1": "",
        "candidate_min_clearance_2": "",
        "candidate_min_margin_1": "",
        "candidate_min_margin_2": "",
        "planned_min_clearance_1": "",
        "planned_min_clearance_2": "",
        "planned_min_margin_1": "",
        "planned_min_margin_2": "",
        "run_dir": run_dir,
        "eval_run_dir": run_dir,
        "trajectory_csv": "",
        "eval_mode": args.eval_mode,
        "candidate_source": args.candidate_source,
        "source_trajectory_csv": "",
        "selected_planned_trajectory_count": 0,
        "rho_astar_waypoint": guide_info.get("rho_astar_waypoint", ""),
        "rho_astar_waypoint_stage2": guide_info.get("rho_astar_waypoint_stage2", args.rho_astar_waypoint_stage2),
        "enable_tail_constraint": guide_info.get("enable_tail_constraint", args.enable_tail_constraint),
        "minco_warm_start_cache": guide_info.get("minco_warm_start_cache", args.minco_warm_start_cache),
        "minco_warm_start_cache_force_reuse": guide_info.get("minco_warm_start_cache_force_reuse", args.minco_warm_start_cache_force_reuse),
        "irl_collision_weight_scale": guide_info.get("irl_collision_weight_scale", args.irl_collision_weight_scale),
        "irl_obs_optimization_margin": guide_info.get("irl_obs_optimization_margin", args.irl_obs_optimization_margin),
        "safety_zone_buffer": guide_info.get("safety_zone_buffer", args.safety_zone_buffer),
        "analytic_safety_radius_1": "",
        "analytic_safety_radius_2": "",
        "s_guide_mode": guide_info.get("s_guide_mode", args.s_guide_mode),
        "s_guide_path": guide_info.get("s_guide_path", args.s_guide_path or ""),
        "s_guide_csv": guide_info.get("s_guide_csv", ""),
        "s_guide_json": guide_info.get("s_guide_json", ""),
        "s_guide_target_clearance_1": guide_info.get("s_guide_target_clearance_1", ""),
        "s_guide_target_clearance_2": guide_info.get("s_guide_target_clearance_2", ""),
        "s_guide_min_clearance_1": guide_info.get("s_guide_min_clearance_1", ""),
        "s_guide_min_clearance_2": guide_info.get("s_guide_min_clearance_2", ""),
        "s_guide_clearance_floor_1": guide_info.get("s_guide_clearance_floor_1", ""),
        "s_guide_clearance_floor_2": guide_info.get("s_guide_clearance_floor_2", ""),
        "s_guide_demo_clearance_1": guide_info.get("s_guide_demo_clearance_1", ""),
        "s_guide_demo_clearance_2": guide_info.get("s_guide_demo_clearance_2", ""),
        "s_guide_demo_floor_margin": guide_info.get("s_guide_demo_floor_margin", ""),
        "candidate_load_error": str(error),
        "points": [],
    }


def result_from_summary(args, eval_run_dir, demo_samples, label, d1, d2, summary, elapsed, result_run_dir=None, guide_info=None):
    summary_typed = coerce_row(summary)
    guide_info = guide_info or {}
    case_dir = os.path.abspath(summary["case_dir"])
    candidate_load_error = ""
    try:
        points, trajectory_csv, source_trajectory_csv, selected_planned_count = load_candidate_points(
            args, case_dir, eval_run_dir, summary
        )
    except Exception as exc:
        points = []
        trajectory_csv = ""
        source_trajectory_csv = summary.get("source_trajectory_csv", "")
        selected_planned_count = int(float(summary.get("planned_traj_count", 0) or 0))
        candidate_load_error = str(exc)

    status = str(summary.get("status", "unknown"))
    completed_wp = int(float(summary.get("completed_waypoints", 0) or 0))
    waypoint_count = int(float(summary.get("waypoint_count", 0) or 0))
    has_points = len(points) >= 2
    if has_points:
        losses = trajectory_loss(demo_samples, points, args.sample_count, args.yaw_weight)
    else:
        losses = {
            "loss": "",
            "position_loss": "",
            "yaw_loss": "",
        }
    feasibility = summarize_feasibility(args, status, summary_typed, eval_run_dir, d1, d2)
    loss_info = compose_outer_loss(args, status, losses, feasibility, completed_wp, waypoint_count, has_points)

    candidate_path_len = summary_typed.get("candidate_path_length", "")
    if candidate_path_len == "" and has_points:
        candidate_path_len = path_length(points)

    return {
        "label": label,
        "d1": d1,
        "d2": d2,
        "loss": loss_info["loss"],
        "loss_mode": loss_info["loss_mode"],
        "imitation_loss": loss_info["imitation_loss"],
        "position_loss": losses["position_loss"],
        "yaw_loss": losses["yaw_loss"],
        "feasibility_loss": loss_info["feasibility_loss"],
        "feasibility_margin_available": feasibility["available"],
        "feasibility_margin_source": feasibility["source"],
        "feasibility_min_clearance_1": feasibility["min_clearance_1"],
        "feasibility_min_clearance_2": feasibility["min_clearance_2"],
        "feasibility_min_margin_1": feasibility["min_margin_1"],
        "feasibility_min_margin_2": feasibility["min_margin_2"],
        "feasibility_violation_1": feasibility["violation_1"],
        "feasibility_violation_2": feasibility["violation_2"],
        "failure_status_penalty_applied": loss_info["failure_status_penalty_applied"],
        "status": status,
        "completed_waypoints": completed_wp,
        "waypoint_count": waypoint_count,
        "failure_count": int(float(summary.get("failure_count", 0) or 0)),
        "failure_reasons": summary.get("failure_reasons", ""),
        "executed_path_length": summary_typed.get("executed_path_length", ""),
        "executed_min_clearance_1": summary_typed.get("executed_min_clearance_1", ""),
        "executed_min_clearance_2": summary_typed.get("executed_min_clearance_2", ""),
        "executed_min_margin_1": summary_typed.get("executed_min_margin_1", ""),
        "executed_min_margin_2": summary_typed.get("executed_min_margin_2", ""),
        "candidate_path_length": candidate_path_len,
        "candidate_min_clearance_1": summary_typed.get("candidate_min_clearance_1", ""),
        "candidate_min_clearance_2": summary_typed.get("candidate_min_clearance_2", ""),
        "candidate_min_margin_1": summary_typed.get("candidate_min_margin_1", ""),
        "candidate_min_margin_2": summary_typed.get("candidate_min_margin_2", ""),
        "planned_min_clearance_1": summary_typed.get("planned_min_clearance_1", ""),
        "planned_min_clearance_2": summary_typed.get("planned_min_clearance_2", ""),
        "planned_min_margin_1": summary_typed.get("planned_min_margin_1", ""),
        "planned_min_margin_2": summary_typed.get("planned_min_margin_2", ""),
        "run_dir": result_run_dir or eval_run_dir,
        "eval_run_dir": eval_run_dir,
        "trajectory_csv": os.path.abspath(trajectory_csv) if trajectory_csv else "",
        "eval_mode": args.eval_mode,
        "candidate_source": args.candidate_source,
        "source_trajectory_csv": os.path.abspath(source_trajectory_csv) if source_trajectory_csv else "",
        "selected_planned_trajectory_count": selected_planned_count,
        "rho_astar_waypoint": guide_info.get("rho_astar_waypoint", ""),
        "rho_astar_waypoint_stage2": guide_info.get("rho_astar_waypoint_stage2", args.rho_astar_waypoint_stage2),
        "enable_tail_constraint": summary_typed.get("enable_tail_constraint", guide_info.get("enable_tail_constraint", args.enable_tail_constraint)),
        "minco_warm_start_cache": guide_info.get("minco_warm_start_cache", args.minco_warm_start_cache),
        "minco_warm_start_cache_force_reuse": summary_typed.get("minco_warm_start_cache_force_reuse", guide_info.get("minco_warm_start_cache_force_reuse", args.minco_warm_start_cache_force_reuse)),
        "irl_collision_weight_scale": guide_info.get("irl_collision_weight_scale", args.irl_collision_weight_scale),
        "irl_obs_optimization_margin": summary_typed.get("irl_obs_optimization_margin", guide_info.get("irl_obs_optimization_margin", args.irl_obs_optimization_margin)),
        "safety_zone_buffer": summary_typed.get("safety_zone_buffer", guide_info.get("safety_zone_buffer", args.safety_zone_buffer)),
        "analytic_safety_radius_1": summary_typed.get("analytic_safety_radius_1", ""),
        "analytic_safety_radius_2": summary_typed.get("analytic_safety_radius_2", ""),
        "duration_sec": elapsed,
        "s_guide_mode": guide_info.get("s_guide_mode", args.s_guide_mode),
        "s_guide_path": guide_info.get("s_guide_path", args.s_guide_path or ""),
        "s_guide_csv": guide_info.get("s_guide_csv", ""),
        "s_guide_json": guide_info.get("s_guide_json", ""),
        "s_guide_target_clearance_1": guide_info.get("s_guide_target_clearance_1", ""),
        "s_guide_target_clearance_2": guide_info.get("s_guide_target_clearance_2", ""),
        "s_guide_min_clearance_1": guide_info.get("s_guide_min_clearance_1", ""),
        "s_guide_min_clearance_2": guide_info.get("s_guide_min_clearance_2", ""),
        "s_guide_clearance_floor_1": guide_info.get("s_guide_clearance_floor_1", ""),
        "s_guide_clearance_floor_2": guide_info.get("s_guide_clearance_floor_2", ""),
        "s_guide_demo_clearance_1": guide_info.get("s_guide_demo_clearance_1", ""),
        "s_guide_demo_clearance_2": guide_info.get("s_guide_demo_clearance_2", ""),
        "s_guide_demo_floor_margin": guide_info.get("s_guide_demo_floor_margin", ""),
        "candidate_load_error": candidate_load_error,
        "points": points,
    }


def evaluate_parameters(args, run_dir, demo_samples, label, iteration, d1, d2):
    eval_results_dir = os.path.join(run_dir, "evaluations")
    os.makedirs(eval_results_dir, exist_ok=True)
    eval_name = "iter_{:02d}_{}_d1_{:.3f}_d2_{:.3f}".format(iteration, label, d1, d2)
    eval_run_dir = os.path.join(eval_results_dir, eval_name)
    os.makedirs(eval_run_dir, exist_ok=True)

    cmd, guide_info = build_eval_command(args, eval_results_dir, eval_name, d1, d2, iteration)
    stdout_log = os.path.join(eval_run_dir, "evaluation_stdout.log")
    if getattr(args, "optimizer_log_mode", "concise") == "verbose":
        print("[fd_irl] eval iter={} label={} d1={:.4f} d2={:.4f}".format(iteration, label, d1, d2), flush=True)
    started = time.time()
    completed = run_logged_command(cmd, stdout_log, tee_stdout=args.tee_eval_stdout)
    elapsed = time.time() - started

    if completed.returncode != 0:
        result = failed_eval(label, d1, d2, eval_run_dir, "returncode {}".format(completed.returncode), args, guide_info)
        result["duration_sec"] = elapsed
        write_eval_result(eval_run_dir, result)
        return result

    try:
        summary = load_single_summary(eval_run_dir)
        result = result_from_summary(args, eval_run_dir, demo_samples, label, d1, d2, summary, elapsed, guide_info=guide_info)
    except Exception as exc:
        result = failed_eval(label, d1, d2, eval_run_dir, exc, args, guide_info)
        result["duration_sec"] = elapsed
        write_eval_result(eval_run_dir, result)
        return result

    write_eval_result(eval_run_dir, result)
    if getattr(args, "optimizer_log_mode", "concise") == "verbose":
        print("[fd_irl] eval done label={} loss={:.6f} status={} waypoints={}/{}".format(
            label, result["loss"], result["status"], result["completed_waypoints"], result["waypoint_count"]
        ), flush=True)
    return result


def evaluate_parameter_batch(args, run_dir, demo_samples, iteration, eval_specs):
    eval_results_dir = os.path.join(run_dir, "evaluations")
    os.makedirs(eval_results_dir, exist_ok=True)
    eval_name = "iter_{:02d}_single_shot_batch".format(iteration)
    eval_run_dir = os.path.join(eval_results_dir, eval_name)
    os.makedirs(eval_run_dir, exist_ok=True)

    cases_text = ";".join([format_case_pair(d1, d2) for _, d1, d2 in eval_specs])
    cmd, guide_info = build_single_shot_eval_command(
        args,
        eval_results_dir,
        eval_name,
        cases_text,
        eval_run_dir,
        iteration=iteration,
    )
    stdout_log = os.path.join(eval_run_dir, "evaluation_stdout.log")
    if getattr(args, "optimizer_log_mode", "concise") == "verbose":
        print(
            "[fd_irl] single_shot batch iter={} cases={}".format(iteration, len(eval_specs)),
            flush=True,
        )
    started = time.time()
    completed = run_logged_command(cmd, stdout_log, tee_stdout=args.tee_eval_stdout)
    elapsed = time.time() - started

    results = {}
    if completed.returncode != 0:
        for label, d1, d2 in eval_specs:
            label_dir = os.path.join(eval_run_dir, "fd_result_{}".format(label))
            os.makedirs(label_dir, exist_ok=True)
            result = failed_eval(label, d1, d2, label_dir, "returncode {}".format(completed.returncode), args, guide_info)
            result["duration_sec"] = elapsed
            result["eval_run_dir"] = eval_run_dir
            write_eval_result(label_dir, result)
            results[label] = result
        return results

    try:
        rows = load_summary_rows(eval_run_dir)
    except Exception as exc:
        rows = []
        load_error = exc
    else:
        load_error = None

    for idx, (label, d1, d2) in enumerate(eval_specs):
        if load_error is not None or idx >= len(rows):
            label_dir = os.path.join(eval_run_dir, "fd_result_{}".format(label))
            os.makedirs(label_dir, exist_ok=True)
            error = load_error or "missing summary row {}".format(idx)
            result = failed_eval(label, d1, d2, label_dir, error, args, guide_info)
            result["duration_sec"] = elapsed
            result["eval_run_dir"] = eval_run_dir
            write_eval_result(label_dir, result)
            results[label] = result
            continue

        row = rows[idx]
        result_dir = os.path.abspath(row.get("case_dir") or eval_run_dir)
        try:
            result = result_from_summary(
                args,
                eval_run_dir,
                demo_samples,
                label,
                d1,
                d2,
                row,
                elapsed,
                result_run_dir=result_dir,
                guide_info=guide_info,
            )
        except Exception as exc:
            label_dir = os.path.join(eval_run_dir, "fd_result_{}".format(label))
            os.makedirs(label_dir, exist_ok=True)
            result = failed_eval(label, d1, d2, label_dir, exc, args, guide_info)
            result["duration_sec"] = elapsed
            result["eval_run_dir"] = eval_run_dir
        write_eval_result(result["run_dir"], result)
        if getattr(args, "optimizer_log_mode", "concise") == "verbose":
            print("[fd_irl] eval done label={} loss={:.6f} status={} waypoints={}/{}".format(
                label,
                result["loss"],
                result["status"],
                result["completed_waypoints"],
                result["waypoint_count"],
            ), flush=True)
        results[label] = result

    return results


def visualize_result_minco_trajectories(viz, iteration, result):
    source_csv = result.get("source_trajectory_csv")
    if not source_csv or not os.path.exists(source_csv):
        return
    try:
        planned = read_planned_trajectories(source_csv)
    except Exception as exc:
        print("[fd_irl] warning: failed to visualize MINCO trajectories from {}: {}".format(
            source_csv,
            exc,
        ), flush=True)
        return
    trajectories = []
    for traj in planned:
        trajectories.append({
            "trajectory_id": traj.get("trajectory_id", ""),
            "points": [
                {"position": [point["x"], point["y"], point["z"]]}
                for point in traj.get("points", [])
            ],
        })
    viz.add_minco_trajectories(
        iteration,
        result.get("label", ""),
        result.get("d1", 0.0),
        result.get("d2", 0.0),
        trajectories,
        selected_index=None,
    )


def load_single_shot_eval_module(script_path):
    script_path = os.path.abspath(script_path)
    module_name = "_airgrasp_minco_irl_single_shot_eval"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load single-shot evaluator module: {}".format(script_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PersistentSingleShotEvaluator:
    def __init__(self, args, run_dir, scene, initial_d1, initial_d2):
        self.args = args
        self.run_dir = os.path.abspath(run_dir)
        self.scene = scene
        self.module = load_single_shot_eval_module(args.single_shot_script)
        self.persistent_dir = os.path.join(self.run_dir, "persistent_single_shot_planner")
        os.makedirs(self.persistent_dir, exist_ok=True)
        self.processes = []
        self.client = None
        self.rows_by_eval_dir = {}
        self.case_seq = 0
        self.map_ready = False

        self.eval_args = self._build_eval_args(args)
        self.eval_scene = self.module.load_scene(self.eval_args.scene_config)
        self.s_guide_path, self.s_guide_metadata = self.module.prepare_s_guide(
            self.eval_args,
            self.persistent_dir,
            self.eval_scene,
            [(initial_d1, initial_d2)],
        )
        self.guide_info = self._build_guide_info(args)

        self._write_metadata(args, initial_d1, initial_d2)
        try:
            print(
                "[fd_irl] starting persistent single-shot planner: rhoAstarWaypoint={:.6f}, stage2={:.6f}, warm_start_cache={} force_reuse={}".format(
                    self.eval_args.rho_astar_waypoint,
                    self.eval_args.rho_astar_waypoint_stage2,
                    self.eval_args.minco_warm_start_cache,
                    self.eval_args.minco_warm_start_cache_force_reuse,
                ),
                flush=True,
            )
            self.processes = self.module.start_processes(
                self.eval_args,
                self.persistent_dir,
                self.eval_scene,
                self.s_guide_path,
                [(initial_d1, initial_d2)],
            )
            self.rospy = ensure_rospy_node("airgrasp_fd_irl_optimizer")
            self.client = self.module.SingleShotClient(self.eval_scene, self.eval_args.fixed_odom_hz)
            self.client.publish_hold(True)

            if not self.client.wait_for_connections(self.eval_args.startup_timeout):
                self.rospy.logwarn("[fd_irl] persistent planner topic connections timed out.")
            if not self.client.wait_for_odom_at_start(
                self.eval_args.start_tolerance,
                self.eval_args.startup_timeout,
            ):
                self.rospy.logwarn("[fd_irl] persistent fixed odom did not settle near start.")
            self.map_ready = self.module.wait_for_map(self.eval_args.map_timeout)
            if not self.map_ready:
                self.rospy.logwarn("[fd_irl] persistent /sdf_map/esdf did not arrive before timeout.")
        except Exception:
            self.shutdown()
            raise

    def _build_eval_args(self, args):
        eval_args = self.module.build_arg_parser().parse_args([])
        eval_args.scene_config = os.path.abspath(args.scene_config)
        eval_args.results_dir = os.path.join(self.run_dir, "evaluations")
        eval_args.run_name = "persistent_single_shot_planner"
        eval_args.sphere_radius = args.sphere_radius
        eval_args.obstacles_inflation = args.obstacles_inflation
        eval_args.safety_zone_buffer = args.safety_zone_buffer
        eval_args.rho_astar_waypoint = rho_astar_waypoint_for_iteration(args, 0)
        eval_args.rho_astar_waypoint_stage2 = args.rho_astar_waypoint_stage2
        eval_args.enable_tail_constraint = args.enable_tail_constraint
        eval_args.minco_warm_start_cache = args.minco_warm_start_cache
        eval_args.minco_warm_start_cache_force_reuse = args.minco_warm_start_cache_force_reuse
        eval_args.irl_collision_weight_scale = args.irl_collision_weight_scale
        eval_args.irl_obs_optimization_margin = args.irl_obs_optimization_margin
        eval_args.master_timeout = args.master_timeout
        eval_args.startup_timeout = args.startup_timeout
        eval_args.map_timeout = args.map_timeout
        eval_args.trajectory_timeout = args.trajectory_timeout
        eval_args.reset_settle_sec = args.reset_settle_sec
        eval_args.case_settle_sec = args.case_settle_sec
        eval_args.cleanup_sec = args.cleanup_sec
        eval_args.fixed_odom_hz = args.fixed_odom_hz
        eval_args.start_tolerance = args.start_tolerance
        eval_args.s_guide_enable = args.s_guide_enable
        eval_args.s_guide_bypass_astar = args.s_guide_bypass_astar
        eval_args.s_guide_endpoint_tolerance = args.s_guide_endpoint_tolerance
        eval_args.s_guide_mode = args.s_guide_mode
        eval_args.s_guide_path = args.s_guide_path
        eval_args.s_guide_clearance_buffer = args.s_guide_clearance_buffer
        eval_args.s_guide_min_clearance = args.s_guide_min_clearance
        eval_args.s_guide_max_clearance = args.s_guide_max_clearance
        eval_args.use_minco_final_trajectory_visualizer = False
        eval_args.minco_reference_csv = ""
        eval_args.world_launch = args.world_launch
        eval_args.planning_launch = args.planning_launch
        eval_args.keep_launch_on_failure = args.keep_eval_launch_on_failure
        eval_args.stream_planning_debug = args.stream_minco_debug
        eval_args.stream_planning_debug_stride = args.stream_minco_debug_stride
        return eval_args

    def _build_guide_info(self, args):
        guide_info = {
            "s_guide_mode": args.s_guide_mode,
            "s_guide_path": self.s_guide_path,
            "rho_astar_waypoint": self.eval_args.rho_astar_waypoint,
            "rho_astar_waypoint_stage2": args.rho_astar_waypoint_stage2,
            "enable_tail_constraint": args.enable_tail_constraint,
            "minco_warm_start_cache": args.minco_warm_start_cache,
            "minco_warm_start_cache_force_reuse": args.minco_warm_start_cache_force_reuse,
            "irl_collision_weight_scale": args.irl_collision_weight_scale,
            "irl_obs_optimization_margin": args.irl_obs_optimization_margin,
            "safety_zone_buffer": args.safety_zone_buffer,
        }
        initial_info = getattr(args, "initial_s_guide_info", None)
        if initial_info:
            guide_info.update({
                "s_guide_mode": initial_info.get("s_guide_mode", "fixed_initial"),
                "s_guide_csv": initial_info.get("s_guide_csv", ""),
                "s_guide_json": initial_info.get("s_guide_json", ""),
                "s_guide_target_clearance_1": initial_info.get("target_clearance_1", ""),
                "s_guide_target_clearance_2": initial_info.get("target_clearance_2", ""),
                "s_guide_min_clearance_1": initial_info.get("guide_min_clearance_1", ""),
                "s_guide_min_clearance_2": initial_info.get("guide_min_clearance_2", ""),
                "s_guide_clearance_floor_1": initial_info.get("clearance_floor_1", ""),
                "s_guide_clearance_floor_2": initial_info.get("clearance_floor_2", ""),
                "s_guide_demo_clearance_1": initial_info.get("s_guide_demo_clearance_1", ""),
                "s_guide_demo_clearance_2": initial_info.get("s_guide_demo_clearance_2", ""),
                "s_guide_demo_floor_margin": initial_info.get("s_guide_demo_floor_margin", ""),
            })
        elif self.s_guide_metadata:
            guide_info.update({
                "s_guide_csv": self.s_guide_metadata.get("s_guide_csv", ""),
                "s_guide_json": self.s_guide_metadata.get("s_guide_json", ""),
            })
        return guide_info

    def _write_metadata(self, args, initial_d1, initial_d2):
        metadata = {
            "created_at": _dt.datetime.now().isoformat(),
            "mode": "persistent_single_shot_planned",
            "scene_config": self.eval_args.scene_config,
            "initial_case": {"d1": initial_d1, "d2": initial_d2},
            "sphere_radius": self.eval_args.sphere_radius,
            "obstacles_inflation": self.eval_args.obstacles_inflation,
            "safety_zone_buffer": self.eval_args.safety_zone_buffer,
            "rho_astar_waypoint": self.eval_args.rho_astar_waypoint,
            "rho_astar_waypoint_stage2": self.eval_args.rho_astar_waypoint_stage2,
            "enable_tail_constraint": self.eval_args.enable_tail_constraint,
            "rho_astar_waypoint_schedule": args.rho_astar_waypoint_schedule,
            "rho_astar_waypoint_schedule_note": (
                "Persistent planner reads rhoAstarWaypoint at launch; schedules only affect metadata."
                if args.rho_astar_waypoint_schedule_values else ""
            ),
            "minco_warm_start_cache": self.eval_args.minco_warm_start_cache,
            "minco_warm_start_cache_force_reuse": self.eval_args.minco_warm_start_cache_force_reuse,
            "irl_collision_weight_scale": self.eval_args.irl_collision_weight_scale,
            "irl_obs_optimization_margin": self.eval_args.irl_obs_optimization_margin,
            "start_position": self.eval_scene["start_position"],
            "goal_position": self.eval_scene["goal_position"],
            "reference_path": self.eval_scene["reference_path"],
            "s_guide_enable": self.eval_args.s_guide_enable,
            "s_guide_bypass_astar": self.eval_args.s_guide_bypass_astar,
            "s_guide_endpoint_tolerance": self.eval_args.s_guide_endpoint_tolerance,
            "s_guide_mode": self.eval_args.s_guide_mode,
            "s_guide_path": self.s_guide_path,
            "s_guide_metadata": self.s_guide_metadata,
            "trajectory_selection": self.eval_args.trajectory_selection,
        }
        with open(os.path.join(self.persistent_dir, "run_metadata.json"), "w") as f:
            json.dump(metadata, f, indent=2)

    def _link_persistent_log(self, eval_run_dir):
        src = os.path.join(self.persistent_dir, "planning.log")
        dst = os.path.join(eval_run_dir, "planning.log")
        if os.path.lexists(dst):
            try:
                os.remove(dst)
            except OSError:
                pass
        try:
            os.symlink(src, dst)
        except OSError:
            pass
        return src, dst

    def _append_eval_stdout(self, eval_run_dir, text):
        with open(os.path.join(eval_run_dir, "evaluation_stdout.log"), "a") as f:
            f.write(text.rstrip() + "\n")

    def evaluate_batch(self, args, run_dir, demo_samples, iteration, eval_specs, viz=None):
        eval_results_dir = os.path.join(run_dir, "evaluations")
        os.makedirs(eval_results_dir, exist_ok=True)
        eval_name = "iter_{:02d}_single_shot_persistent_batch".format(iteration)
        eval_run_dir = os.path.join(eval_results_dir, eval_name)
        os.makedirs(eval_run_dir, exist_ok=True)
        planning_log, planning_log_link = self._link_persistent_log(eval_run_dir)

        self._append_eval_stdout(
            eval_run_dir,
            "[fd_irl] persistent single_shot batch iter={} cases={}".format(iteration, len(eval_specs)),
        )
        if getattr(args, "optimizer_log_mode", "concise") == "verbose":
            print(
                "[fd_irl] persistent single_shot batch iter={} cases={}".format(iteration, len(eval_specs)),
                flush=True,
            )

        rows = []
        results = {}
        for idx, (label, d1, d2) in enumerate(eval_specs):
            case_dir = os.path.join(eval_run_dir, "case_{:02d}_{}_d1_{:.3f}_d2_{:.3f}".format(
                idx,
                label,
                d1,
                d2,
            ))
            os.makedirs(case_dir, exist_ok=True)
            start_wall = time.time()
            case_result = None
            try:
                self._append_eval_stdout(
                    eval_run_dir,
                    "[fd_irl] case {} label={} d1={:.6f} d2={:.6f}".format(idx, label, d1, d2),
                )
                self.client.publish_hold(True)
                self.rospy.sleep(self.eval_args.reset_settle_sec)
                self.client.publish_safety(d1, d2, self.eval_args.sphere_radius)
                self.client.begin_case()
                try:
                    self.case_seq += 1
                    self.client.publish_goal(self.case_seq)
                    self.rospy.sleep(0.1)
                    self.client.publish_hold(False)
                    first_trajectory = self.client.wait_for_first_trajectory(self.eval_args.trajectory_timeout)
                    if (
                        first_trajectory is not None and
                        self.eval_args.trajectory_selection == "best_quality"
                    ):
                        self.rospy.sleep(max(0.0, self.eval_args.trajectory_collection_sec))
                    self.client.end_case()
                    self.client.publish_hold(True)
                    self.rospy.sleep(self.eval_args.case_settle_sec)
                finally:
                    duration_sec = time.time() - start_wall
                    self.client.end_case()

                snapshot = self.client.snapshot()
                quality_rows = self.module.evaluate_trajectory_qualities(
                    snapshot["trajectories"],
                    self.eval_scene,
                    self.eval_args.sphere_radius,
                    d1,
                    d2,
                    self.eval_args,
                )
                trajectory, selection_meta = self.module.select_trajectory(
                    snapshot["trajectories"],
                    quality_rows,
                    self.eval_args.trajectory_selection,
                )
                if viz is not None:
                    selected_index = finite_float(selection_meta.get("selected_trajectory_index"))
                    if selected_index is not None:
                        selected_index = int(selected_index)
                    viz.add_minco_trajectories(
                        iteration,
                        label,
                        d1,
                        d2,
                        snapshot["trajectories"],
                        selected_index=selected_index,
                    )
                case_meta = {
                    "case_id": idx,
                    "label": label,
                    "d1": d1,
                    "d2": d2,
                    "sphere_radius": self.eval_args.sphere_radius,
                    "success": trajectory is not None and (
                        not self.module.quality_required(self.eval_args.trajectory_selection) or
                        selection_meta.get("quality_pass", False)
                    ),
                    "duration_sec": duration_sec,
                    "trajectory_selection": selection_meta,
                    "persistent_case_seq": self.case_seq,
                }
                self.module.write_case_outputs(
                    case_dir,
                    case_meta,
                    trajectory,
                    snapshot,
                    quality_rows,
                    selection_meta,
                )
                row = self.module.summarize_case(
                    idx,
                    d1,
                    d2,
                    self.eval_args.sphere_radius,
                    self.eval_scene,
                    trajectory,
                    snapshot,
                    duration_sec,
                    case_dir,
                    self.map_ready,
                    selection_meta,
                    self.eval_args,
                )
                row["case_id"] = idx
                rows.append(row)
                self.module.write_summary(eval_run_dir, rows)

                invalidate_planning_margin_cache(planning_log, planning_log_link)
                case_result = result_from_summary(
                    args,
                    eval_run_dir,
                    demo_samples,
                    label,
                    d1,
                    d2,
                    row,
                    duration_sec,
                    result_run_dir=case_dir,
                    guide_info=self.guide_info,
                )
            except Exception as exc:
                invalidate_planning_margin_cache(planning_log, planning_log_link)
                label_dir = os.path.join(eval_run_dir, "fd_result_{}".format(label))
                os.makedirs(label_dir, exist_ok=True)
                case_result = failed_eval(label, d1, d2, eval_run_dir, exc, args, self.guide_info)
                case_result["run_dir"] = label_dir
                case_result["eval_run_dir"] = eval_run_dir
                case_result["duration_sec"] = time.time() - start_wall
                self._append_eval_stdout(
                    eval_run_dir,
                    "[fd_irl] case {} label={} failed: {}".format(idx, label, exc),
                )

            write_eval_result(case_result["run_dir"], case_result)
            if getattr(args, "optimizer_log_mode", "concise") == "verbose":
                print("[fd_irl] eval done label={} loss={:.6f} status={} waypoints={}/{}".format(
                    label,
                    case_result["loss"],
                    case_result["status"],
                    case_result["completed_waypoints"],
                    case_result["waypoint_count"],
                ), flush=True)
            results[label] = case_result

        self.rows_by_eval_dir[eval_run_dir] = rows
        return results

    def shutdown(self):
        if self.client is not None:
            try:
                self.client.publish_hold(True)
                self.client.shutdown()
            except Exception:
                pass
            self.client = None
        if self.processes:
            self.module.stop_processes(self.processes)
            self.processes = []
            time.sleep(max(0.0, self.eval_args.cleanup_sec))


def json_safe_eval(result):
    data = dict(result)
    data.pop("points", None)
    return data


def write_eval_result(eval_run_dir, result):
    os.makedirs(eval_run_dir, exist_ok=True)
    with open(os.path.join(eval_run_dir, "fd_eval_result.json"), "w") as f:
        json.dump(json_safe_eval(result), f, indent=2)


def bounded(value, lower, upper):
    return min(max(value, lower), upper)


def finite_difference_step(value, eps, lower, upper):
    plus = bounded(value + eps, lower, upper)
    minus = bounded(value - eps, lower, upper)
    return plus, minus, plus - minus


def is_gradient_usable(result):
    try:
        loss = float(result.get("loss"))
    except (TypeError, ValueError):
        return False
    if not math.isfinite(loss):
        return False
    return gradient_channel(result) != "unusable"


def is_success_loss_usable(result):
    if result.get("status") != "success":
        return False
    try:
        loss = float(result.get("loss"))
    except (TypeError, ValueError):
        return False
    return math.isfinite(loss) and len(result.get("points") or []) >= 2


def is_feasibility_boundary_usable(result):
    if result.get("status") == "success":
        return False
    if result.get("loss_mode") != "feasibility_boundary":
        return False
    if not result.get("feasibility_margin_available"):
        return False
    try:
        loss = float(result.get("loss"))
        feasibility_loss = float(result.get("feasibility_loss"))
    except (TypeError, ValueError):
        return False
    return math.isfinite(loss) and math.isfinite(feasibility_loss)


def gradient_channel(result):
    if is_success_loss_usable(result):
        return "imitation"
    if is_feasibility_boundary_usable(result):
        return "feasibility"
    return "unusable"


def boundary_loss(result):
    try:
        return float(result.get("loss"))
    except (TypeError, ValueError):
        return 0.0


def format_failed_boundary(axis_name, plus_result, plus_value, minus_result, minus_value):
    failed = []
    if not is_gradient_usable(plus_result):
        failed.append("{}_plus:{:.6f}".format(axis_name, plus_value))
    if not is_gradient_usable(minus_result):
        failed.append("{}_minus:{:.6f}".format(axis_name, minus_value))
    return "|".join(failed)


def finite_difference_gradient(axis_name, plus_result, minus_result, plus_value, minus_value, denom):
    plus_channel = gradient_channel(plus_result)
    minus_channel = gradient_channel(minus_result)
    plus_usable = plus_channel != "unusable"
    minus_usable = minus_channel != "unusable"
    if denom <= 1e-9:
        return {
            "gradient": 0.0,
            "mode": "skipped_degenerate_step",
            "plus_usable": plus_usable,
            "minus_usable": minus_usable,
            "failed_boundary": "",
        }
    if plus_channel == "imitation" and minus_channel == "imitation":
        return {
            "gradient": (plus_result["loss"] - minus_result["loss"]) / denom,
            "mode": "central_difference",
            "plus_usable": True,
            "minus_usable": True,
            "failed_boundary": "",
        }
    if plus_channel == "feasibility" and minus_channel == "feasibility":
        return {
            "gradient": (plus_result["loss"] - minus_result["loss"]) / denom,
            "mode": "feasibility_difference",
            "plus_usable": True,
            "minus_usable": True,
            "failed_boundary": "",
        }
    if plus_channel == "feasibility":
        return {
            "gradient": boundary_loss(plus_result) / denom,
            "mode": "boundary_plus_failed",
            "plus_usable": True,
            "minus_usable": minus_usable,
            "failed_boundary": "{}_plus:{:.6f}".format(axis_name, plus_value),
        }
    if minus_channel == "feasibility":
        return {
            "gradient": -boundary_loss(minus_result) / denom,
            "mode": "boundary_minus_failed",
            "plus_usable": plus_usable,
            "minus_usable": True,
            "failed_boundary": "{}_minus:{:.6f}".format(axis_name, minus_value),
        }
    return {
        "gradient": 0.0,
        "mode": "skipped_failed_case",
        "plus_usable": plus_usable,
        "minus_usable": minus_usable,
        "failed_boundary": format_failed_boundary(
            axis_name, plus_result, plus_value, minus_result, minus_value
        ),
    }


def clipped_update(value, gradient, learning_rate, max_step, lower, upper):
    raw_step = -learning_rate * gradient
    step = bounded(raw_step, -max_step, max_step)
    next_value = bounded(value + step, lower, upper)
    return next_value, next_value - value


def loss_value(result):
    try:
        loss = float(result.get("loss"))
    except (TypeError, ValueError):
        return float("inf")
    return loss if math.isfinite(loss) else float("inf")


def update_channel(result):
    if is_success_loss_usable(result):
        return "imitation"
    if is_feasibility_boundary_usable(result):
        return "feasibility"
    return "unusable"


def best_neighbor_update(current, neighbor_specs, min_improvement=1e-9):
    candidates = [("current", current, current["d1"], current["d2"])]
    for label, result in neighbor_specs:
        candidates.append((label, result, result["d1"], result["d2"]))

    success_candidates = [
        item for item in candidates if update_channel(item[1]) == "imitation"
    ]
    feasible_candidates = [
        item for item in candidates if update_channel(item[1]) == "feasibility"
    ]
    pool = success_candidates or feasible_candidates
    if not pool:
        return {
            "next_d1": current["d1"],
            "next_d2": current["d2"],
            "step_d1": 0.0,
            "step_d2": 0.0,
            "accepted_label": "current",
            "accepted_loss": current.get("loss", ""),
            "accepted_channel": "unusable",
            "accepted_reason": "no_usable_neighbor",
        }

    best_label, best_result, best_d1, best_d2 = min(
        pool, key=lambda item: loss_value(item[1])
    )
    current_items = [item for item in pool if item[0] == "current"]
    if current_items:
        _, current_result, current_d1, current_d2 = current_items[0]
        if loss_value(current_result) <= loss_value(best_result) + min_improvement:
            best_label = "current"
            best_result = current_result
            best_d1 = current_d1
            best_d2 = current_d2

    return {
        "next_d1": best_d1,
        "next_d2": best_d2,
        "step_d1": best_d1 - current["d1"],
        "step_d2": best_d2 - current["d2"],
        "accepted_label": best_label,
        "accepted_loss": loss_value(best_result),
        "accepted_channel": update_channel(best_result),
        "accepted_reason": "min_loss_neighbor",
    }


def compact_float(value, digits=4):
    number = finite_float(value)
    if number is None:
        return "-"
    return "{:.{digits}f}".format(number, digits=digits)


def compact_loss(value):
    number = finite_float(value)
    if number is None:
        return "-"
    if abs(number) >= 1000.0:
        return "{:.3e}".format(number)
    return "{:.6f}".format(number)


def compact_text(value, width):
    text = str(value if value not in (None, "") else "-")
    if len(text) <= width:
        return text
    return text[:max(0, width - 1)] + "."


def result_clearance(result, index):
    return first_finite(
        result.get("candidate_min_clearance_{}".format(index)),
        result.get("feasibility_min_clearance_{}".format(index)),
        result.get("planned_min_clearance_{}".format(index)),
        result.get("executed_min_clearance_{}".format(index)),
    )


def result_margin(result, index):
    return first_finite(
        result.get("candidate_min_margin_{}".format(index)),
        result.get("feasibility_min_margin_{}".format(index)),
        result.get("planned_min_margin_{}".format(index)),
        result.get("executed_min_margin_{}".format(index)),
    )


def print_iteration_summary(
    args,
    iteration,
    rho_astar_waypoint,
    current,
    evals,
    g1,
    g2,
    g1_info,
    g2_info,
    update_info,
    next_d1,
    next_d2,
):
    accepted_label = update_info.get("accepted_label", "")
    rows = [
        ("current", current),
        ("d1_plus", evals["d1_plus"]),
        ("d1_minus", evals["d1_minus"]),
        ("d2_plus", evals["d2_plus"]),
        ("d2_minus", evals["d2_minus"]),
    ]

    print("", flush=True)
    print(
        "[fd_irl] iter {}/{} summary | rho(stage1)={} stage2={} | update={}".format(
            iteration,
            args.max_iters - 1,
            compact_float(rho_astar_waypoint, 1),
            compact_float(args.rho_astar_waypoint_stage2, 1),
            args.outer_update_mode,
        ),
        flush=True,
    )
    print(
        "  {:<10} {:>7} {:>7} {:>10} {:<12} {:<11} {:>7} {:>7} {:>7} {:>7} {:>5}".format(
            "case", "d1", "d2", "loss", "status", "channel", "clr1", "clr2", "mrg1", "mrg2", "fail"
        ),
        flush=True,
    )
    for label, result in rows:
        mark = "*" if label == accepted_label else " "
        failure_count = result.get("failure_count", "")
        if failure_count in (None, ""):
            failure_count = 0 if result.get("status") == "success" else "-"
        print(
            "{} {:<10} {:>7} {:>7} {:>10} {:<12} {:<11} {:>7} {:>7} {:>7} {:>7} {:>5}".format(
                mark,
                compact_text(label, 10),
                compact_float(result.get("d1"), 3),
                compact_float(result.get("d2"), 3),
                compact_loss(result.get("loss")),
                compact_text(result.get("status"), 12),
                compact_text(update_channel(result), 11),
                compact_float(result_clearance(result, 1), 3),
                compact_float(result_clearance(result, 2), 3),
                compact_float(result_margin(result, 1), 3),
                compact_float(result_margin(result, 2), 3),
                compact_text(failure_count, 5),
            ),
            flush=True,
        )

    print(
        "  accept={} channel={} reason={} | g=({:+.4f},{:+.4f}) mode=({}, {}) | next=({:.4f},{:.4f})".format(
            update_info.get("accepted_label", "-"),
            update_info.get("accepted_channel", "-"),
            update_info.get("accepted_reason", "-"),
            g1,
            g2,
            g1_info.get("mode", "-"),
            g2_info.get("mode", "-"),
            next_d1,
            next_d2,
        ),
        flush=True,
    )


def write_csv(path, rows, fields):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def write_optimizer_outputs(run_dir, eval_rows, trace_rows):
    eval_fields = [
        "iteration",
        "label",
        "d1",
        "d2",
        "loss",
        "loss_mode",
        "imitation_loss",
        "position_loss",
        "yaw_loss",
        "feasibility_loss",
        "feasibility_margin_available",
        "feasibility_margin_source",
        "feasibility_min_clearance_1",
        "feasibility_min_clearance_2",
        "feasibility_min_margin_1",
        "feasibility_min_margin_2",
        "feasibility_violation_1",
        "feasibility_violation_2",
        "failure_status_penalty_applied",
        "status",
        "completed_waypoints",
        "waypoint_count",
        "failure_count",
        "failure_reasons",
        "executed_path_length",
        "executed_min_clearance_1",
        "executed_min_clearance_2",
        "executed_min_margin_1",
        "executed_min_margin_2",
        "candidate_path_length",
        "candidate_min_clearance_1",
        "candidate_min_clearance_2",
        "candidate_min_margin_1",
        "candidate_min_margin_2",
        "planned_min_clearance_1",
        "planned_min_clearance_2",
        "planned_min_margin_1",
        "planned_min_margin_2",
        "duration_sec",
        "run_dir",
        "eval_run_dir",
        "trajectory_csv",
        "eval_mode",
        "candidate_source",
        "source_trajectory_csv",
        "selected_planned_trajectory_count",
        "rho_astar_waypoint",
        "rho_astar_waypoint_stage2",
        "enable_tail_constraint",
        "minco_warm_start_cache",
        "minco_warm_start_cache_force_reuse",
        "irl_collision_weight_scale",
        "irl_obs_optimization_margin",
        "safety_zone_buffer",
        "analytic_safety_radius_1",
        "analytic_safety_radius_2",
        "s_guide_mode",
        "s_guide_csv",
        "s_guide_json",
        "s_guide_target_clearance_1",
        "s_guide_target_clearance_2",
        "s_guide_min_clearance_1",
        "s_guide_min_clearance_2",
        "s_guide_clearance_floor_1",
        "s_guide_clearance_floor_2",
        "s_guide_demo_clearance_1",
        "s_guide_demo_clearance_2",
        "s_guide_demo_floor_margin",
        "candidate_load_error",
    ]
    trace_fields = [
        "iteration",
        "d1",
        "d2",
        "loss",
        "loss_mode",
        "imitation_loss",
        "position_loss",
        "yaw_loss",
        "feasibility_loss",
        "feasibility_margin_available",
        "feasibility_margin_source",
        "feasibility_min_clearance_1",
        "feasibility_min_clearance_2",
        "feasibility_min_margin_1",
        "feasibility_min_margin_2",
        "feasibility_violation_1",
        "feasibility_violation_2",
        "failure_status_penalty_applied",
        "g1",
        "g2",
        "g1_mode",
        "g2_mode",
        "d1_plus_usable",
        "d1_minus_usable",
        "d2_plus_usable",
        "d2_minus_usable",
        "d1_failed_boundary",
        "d2_failed_boundary",
        "outer_update_mode",
        "accepted_label",
        "accepted_loss",
        "accepted_channel",
        "accepted_reason",
        "next_d1",
        "next_d2",
        "step_d1",
        "step_d2",
        "status",
        "completed_waypoints",
        "waypoint_count",
        "failure_count",
        "executed_path_length",
        "executed_min_clearance_1",
        "executed_min_clearance_2",
        "executed_min_margin_1",
        "executed_min_margin_2",
        "candidate_path_length",
        "candidate_min_clearance_1",
        "candidate_min_clearance_2",
        "candidate_min_margin_1",
        "candidate_min_margin_2",
        "run_dir",
        "eval_run_dir",
        "trajectory_csv",
        "eval_mode",
        "candidate_source",
        "source_trajectory_csv",
        "selected_planned_trajectory_count",
        "rho_astar_waypoint",
        "rho_astar_waypoint_stage2",
        "enable_tail_constraint",
        "minco_warm_start_cache",
        "minco_warm_start_cache_force_reuse",
        "irl_collision_weight_scale",
        "irl_obs_optimization_margin",
        "safety_zone_buffer",
        "analytic_safety_radius_1",
        "analytic_safety_radius_2",
        "s_guide_mode",
        "s_guide_csv",
        "s_guide_json",
        "s_guide_target_clearance_1",
        "s_guide_target_clearance_2",
        "s_guide_min_clearance_1",
        "s_guide_min_clearance_2",
        "s_guide_clearance_floor_1",
        "s_guide_clearance_floor_2",
        "s_guide_demo_clearance_1",
        "s_guide_demo_clearance_2",
        "s_guide_demo_floor_margin",
        "candidate_load_error",
    ]
    write_csv(os.path.join(run_dir, "evaluations.csv"), eval_rows, eval_fields)
    write_csv(os.path.join(run_dir, "optimization_trace.csv"), trace_rows, trace_fields)
    with open(os.path.join(run_dir, "evaluations.json"), "w") as f:
        json.dump(eval_rows, f, indent=2)
    with open(os.path.join(run_dir, "optimization_trace.json"), "w") as f:
        json.dump(trace_rows, f, indent=2)


def save_best(run_dir, current, best):
    if current.get("status") != "success" or not current.get("trajectory_csv"):
        return best
    if best is None or current["loss"] < best["loss"]:
        best = json_safe_eval(current)
        best_path = os.path.join(run_dir, "best_trajectory.csv")
        if current.get("trajectory_csv"):
            shutil.copyfile(current["trajectory_csv"], best_path)
            best["best_trajectory_csv"] = os.path.abspath(best_path)
        with open(os.path.join(run_dir, "best_result.json"), "w") as f:
            json.dump(best, f, indent=2)
    return best


def build_arg_parser():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Run a finite-difference outer-loop optimizer for two-pillar d1/d2 IRL."
    )
    parser.add_argument("--demo-csv", default=None,
                        help="Demo trajectory CSV. Default: newest demo_trajectory.csv under --demo-dir.")
    parser.add_argument("--demo-dir", default=os.path.join(root, "data", "demos"),
                        help="Directory used for auto-selecting the newest collected demo.")
    parser.add_argument("--results-dir", default=os.path.join(root, "results", "runs"))
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--eval-mode", choices=["single_shot"], default="single_shot",
                        help="Evaluation backend. Only one-shot start-to-goal evaluation is supported.")
    parser.add_argument("--persistent-planner", type=parse_bool, default=True,
                        help="Keep the single-shot world/planning launch alive across all FD iterations.")
    parser.add_argument("--single-shot-script",
                        default=os.path.join(root, "scripts", "run_single_shot_planning_eval.py"))
    parser.add_argument("--scene-config", default=os.path.join(root, "config", "two_pillar_scene.yaml"))
    parser.add_argument("--max-iters", type=int, default=5)
    parser.add_argument("--init-d1", type=float, default=DEFAULT_INIT_D1)
    parser.add_argument("--init-d2", type=float, default=DEFAULT_INIT_D2)
    parser.add_argument("--eps", type=float, default=0.02)
    parser.add_argument("--learning-rate", type=float, default=0.30)
    parser.add_argument("--max-step", type=float, default=0.03)
    parser.add_argument("--outer-update-mode", choices=["best_neighbor", "gradient"],
                        default="best_neighbor",
                        help="best_neighbor accepts the evaluated current/plus/minus case with the lowest usable loss; gradient keeps the old finite-difference update.")
    parser.add_argument("--best-neighbor-min-improvement", type=float, default=1e-9,
                        help="Minimum loss improvement required to move away from current in best_neighbor mode.")
    parser.add_argument("--min-d", type=float, default=0.05)
    parser.add_argument("--max-d", type=float, default=DEFAULT_MAX_D)
    parser.add_argument("--sample-count", type=int, default=200)
    parser.add_argument("--yaw-weight", type=float, default=0.0)
    parser.add_argument("--candidate-source", choices=["planned", "executed"], default="planned",
                        help="Candidate trajectory used for imitation loss. Default: planned.")
    parser.add_argument("--planned-selection", choices=["first_by_goal", "last_by_goal", "all"],
                        default="first_by_goal",
                        help="How to reduce planned_trajectories.csv into one candidate path.")
    parser.add_argument("--failure-loss-mode", choices=["constant", "feasibility_boundary"],
                        default="feasibility_boundary",
                        help="How failed evaluations enter the outer loss. feasibility_boundary uses final MINCO margins when available.")
    parser.add_argument("--failure-loss-penalty", type=float, default=10.0)
    parser.add_argument("--missing-goal-penalty", type=float, default=2.0)
    parser.add_argument("--feasibility-loss-weight", type=float, default=300.0,
                        help="Weight for sum(max(0, -margin_j)^2) used by feasibility_boundary mode.")
    parser.add_argument("--failure-status-penalty", type=float, default=0.02,
                        help="Small fixed penalty added to failed evaluations when feasibility_boundary loss is available.")
    parser.add_argument("--sphere-radius", type=float, default=DEFAULT_SPHERE_RADIUS)
    parser.add_argument("--obstacles-inflation", type=float, default=0.40)
    parser.add_argument("--safety-zone-buffer", type=float, default=0.0,
                        help="Extra radius added to pillar_radius + sphere_radius + d_j for safety visualization and analytic checks.")
    parser.add_argument("--rho-astar-waypoint", type=float, default=DEFAULT_RHO_ASTAR_WAYPOINT,
                        help="Static ROS override for planning/rhoAstarWaypoint when the schedule is empty.")
    parser.add_argument("--rho-astar-waypoint-stage2", type=float, default=DEFAULT_RHO_ASTAR_WAYPOINT_STAGE2,
                        help="ROS override for planning/rhoAstarWaypointStage2. Use 0 to keep guide tracking only in warm-start stage.")
    parser.add_argument("--rho-astar-waypoint-schedule", default=DEFAULT_RHO_ASTAR_WAYPOINT_SCHEDULE,
                        help="Comma-separated per-iteration rhoAstarWaypoint schedule. The last value is held. Use '' to disable.")
    parser.add_argument("--enable-tail-constraint", type=parse_bool, default=False,
                        help="IRL experiment override for planning/enable_tail_constraint.")
    parser.add_argument("--minco-warm-start-cache", type=parse_bool, default=DEFAULT_MINCO_WARM_START_CACHE,
                        help="Reuse the first stage-1 warm-start inside each planner process; later cases start directly from that cache and run stage2.")
    parser.add_argument("--minco-warm-start-cache-force-reuse", type=parse_bool,
                        default=DEFAULT_MINCO_WARM_START_CACHE_FORCE_REUSE,
                        help="Force later cases to reuse the stored stage-1 warm-start when dimensions match; useful for fixed-guide IRL evaluation.")
    parser.add_argument("--irl-collision-weight-scale", type=float, default=3.0,
                        help="ROS override for planning/irl_collision_weight_scale; scales only the IRL obstacle cost.")
    parser.add_argument("--irl-obs-optimization-margin", type=float, default=0.05,
                        help="Extra clearance used only by the MINCO IRL obstacle cost; hard checks still use raw d1/d2.")
    parser.add_argument("--master-timeout", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=35.0)
    parser.add_argument("--map-timeout", type=float, default=20.0)
    parser.add_argument("--goal-timeout", type=float, default=90.0)
    parser.add_argument("--trajectory-timeout", type=float, default=45.0)
    parser.add_argument("--goal-republish-sec", type=float, default=1.0)
    parser.add_argument("--post-goal-settle-sec", type=float, default=0.5)
    parser.add_argument("--reset-settle-sec", type=float, default=1.0)
    parser.add_argument("--case-settle-sec", type=float, default=2.0)
    parser.add_argument("--cleanup-sec", type=float, default=2.0)
    parser.add_argument("--fixed-odom-hz", type=float, default=50.0)
    parser.add_argument("--start-tolerance", type=float, default=0.05)
    parser.add_argument("--s-guide-enable", type=parse_bool, default=True)
    parser.add_argument("--s-guide-bypass-astar", type=parse_bool, default=True)
    parser.add_argument("--s-guide-endpoint-tolerance", type=float, default=0.45)
    parser.add_argument("--s-guide-mode", choices=["fixed", "adaptive"], default="fixed",
                        help="fixed uses --s-guide-path/reference path; adaptive materializes one fixed guide from init d1/d2 before optimization.")
    parser.add_argument("--s-guide-path", default=None,
                        help="Override fixed guide path passed to the single-shot evaluator.")
    parser.add_argument("--s-guide-clearance-buffer", type=float, default=DEFAULT_CLEARANCE_BUFFER)
    parser.add_argument("--s-guide-min-clearance", type=float, default=DEFAULT_MIN_CLEARANCE)
    parser.add_argument("--s-guide-max-clearance", type=float, default=DEFAULT_MAX_CLEARANCE)
    parser.add_argument("--s-guide-demo-floor-enable", type=parse_bool, default=False,
                        help="Keep the initial materialized adaptive guide outside the demo clearance envelope.")
    parser.add_argument("--s-guide-demo-floor-start-margin", type=float, default=0.10,
                        help="Extra clearance outside the demo envelope for the initial materialized adaptive guide.")
    parser.add_argument("--s-guide-demo-floor-end-margin", type=float, default=0.0,
                        help="Deprecated; ignored because adaptive guide is fixed once before optimization.")
    parser.add_argument("--world-launch", default=None)
    parser.add_argument("--planning-launch", default=None)
    parser.add_argument("--mock-odom-script", default=None)
    parser.add_argument("--keep-eval-launch-on-failure", action="store_true")
    parser.add_argument("--disable-visualization", action="store_true")
    parser.add_argument("--marker-topic", default="/scene/irl_fd_optimizer_markers")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--viz-stride", type=int, default=3)
    parser.add_argument("--visualize-minco-trajectories", type=parse_bool, default=True,
                        help="Visualize every planned trajectory published by MINCO during FD evaluation.")
    parser.add_argument("--viz-minco-history-limit", type=int, default=300,
                        help="Maximum raw MINCO trajectories kept in RViz. Use 0 for unlimited.")
    parser.add_argument("--hold-after-sec", type=float, default=8.0)
    parser.add_argument("--optimizer-log-mode", choices=["concise", "verbose"], default="concise",
                        help="Terminal logging level. concise prints one compact table per FD iteration; verbose restores per-evaluation progress lines.")
    parser.add_argument("--tee-eval-stdout", type=parse_bool, default=False,
                        help="Mirror evaluation stdout to this terminal while still saving evaluation_stdout.log.")
    parser.add_argument("--stream-minco-debug", type=parse_bool, default=False,
                        help="Ask single-shot evaluations to forward sampled [MINCO-IRL] lines from planning.log.")
    parser.add_argument("--stream-minco-debug-stride", type=int, default=20,
                        help="Forward every Nth [MINCO-IRL] line to this terminal; planning.log keeps all lines.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    args.rho_astar_waypoint_schedule_values = parse_float_schedule(args.rho_astar_waypoint_schedule)
    if args.max_iters < 0:
        raise ValueError("--max-iters must be non-negative.")
    if args.min_d >= args.max_d:
        raise ValueError("--min-d must be less than --max-d.")
    if args.candidate_source != "planned":
        raise ValueError("--eval-mode single_shot only supports --candidate-source planned.")

    run_name = args.run_name or "irl_fd_{}".format(now_tag())
    run_dir = os.path.abspath(os.path.join(args.results_dir, run_name))
    os.makedirs(run_dir, exist_ok=True)

    demo_csv = args.demo_csv or latest_demo_csv(args.demo_dir)
    if demo_csv is None:
        raise RuntimeError("No demo CSV found. Run collect_sim_demo.py first or pass --demo-csv.")
    demo_csv = os.path.abspath(demo_csv)
    demo_points = read_trajectory(demo_csv)
    demo_samples = resample_by_path_length(demo_points, args.sample_count)
    scene = load_scene(args.scene_config)
    demo_clearance = {"min_clearance_1": None, "min_clearance_2": None}
    if args.s_guide_demo_floor_enable:
        demo_clearance = trajectory_clearance_stats(demo_points, scene, args.sphere_radius)
    args.s_guide_demo_clearance_1 = demo_clearance["min_clearance_1"]
    args.s_guide_demo_clearance_2 = demo_clearance["min_clearance_2"]
    requested_s_guide_mode = args.s_guide_mode
    initial_s_guide_info = materialize_initial_s_guide(args, run_dir)

    metadata = {
        "created_at": _dt.datetime.now().isoformat(),
        "demo_csv": demo_csv,
        "demo_dir": os.path.abspath(args.demo_dir),
        "run_dir": run_dir,
        "scene_config": os.path.abspath(args.scene_config),
        "eval_mode": args.eval_mode,
        "persistent_planner": args.persistent_planner,
        "single_shot_script": os.path.abspath(args.single_shot_script),
        "max_iters": args.max_iters,
        "init_d1": args.init_d1,
        "init_d2": args.init_d2,
        "eps": args.eps,
        "learning_rate": args.learning_rate,
        "max_step": args.max_step,
        "outer_update_mode": args.outer_update_mode,
        "best_neighbor_min_improvement": args.best_neighbor_min_improvement,
        "min_d": args.min_d,
        "max_d": args.max_d,
        "sample_count": args.sample_count,
        "yaw_weight": args.yaw_weight,
        "candidate_source": args.candidate_source,
        "planned_selection": args.planned_selection,
        "failure_loss_mode": args.failure_loss_mode,
        "failure_loss_penalty": args.failure_loss_penalty,
        "missing_goal_penalty": args.missing_goal_penalty,
        "feasibility_loss_weight": args.feasibility_loss_weight,
        "failure_status_penalty": args.failure_status_penalty,
        "sphere_radius": args.sphere_radius,
        "obstacles_inflation": args.obstacles_inflation,
        "safety_zone_buffer": args.safety_zone_buffer,
        "rho_astar_waypoint": args.rho_astar_waypoint,
        "rho_astar_waypoint_stage2": args.rho_astar_waypoint_stage2,
        "enable_tail_constraint": args.enable_tail_constraint,
        "rho_astar_waypoint_schedule": args.rho_astar_waypoint_schedule,
        "rho_astar_waypoint_schedule_values": args.rho_astar_waypoint_schedule_values,
        "minco_warm_start_cache": args.minco_warm_start_cache,
        "minco_warm_start_cache_force_reuse": args.minco_warm_start_cache_force_reuse,
        "irl_collision_weight_scale": args.irl_collision_weight_scale,
        "irl_obs_optimization_margin": args.irl_obs_optimization_margin,
        "s_guide_enable": args.s_guide_enable,
        "s_guide_bypass_astar": args.s_guide_bypass_astar,
        "s_guide_endpoint_tolerance": args.s_guide_endpoint_tolerance,
        "s_guide_requested_mode": requested_s_guide_mode,
        "s_guide_mode": args.s_guide_mode,
        "s_guide_path": args.s_guide_path,
        "initial_s_guide": initial_s_guide_info,
        "s_guide_clearance_buffer": args.s_guide_clearance_buffer,
        "s_guide_min_clearance": args.s_guide_min_clearance,
        "s_guide_max_clearance": args.s_guide_max_clearance,
        "s_guide_demo_floor_enable": args.s_guide_demo_floor_enable,
        "s_guide_demo_floor_start_margin": args.s_guide_demo_floor_start_margin,
        "s_guide_demo_floor_end_margin": args.s_guide_demo_floor_end_margin,
        "s_guide_demo_clearance_1": args.s_guide_demo_clearance_1,
        "s_guide_demo_clearance_2": args.s_guide_demo_clearance_2,
        "marker_topic": None if args.disable_visualization else args.marker_topic,
        "visualize_minco_trajectories": False if args.disable_visualization else args.visualize_minco_trajectories,
        "viz_minco_history_limit": args.viz_minco_history_limit,
    }
    with open(os.path.join(run_dir, "optimizer_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    if args.max_iters == 0:
        write_optimizer_outputs(run_dir, [], [])
        final_result = {
            "run_dir": run_dir,
            "demo_csv": demo_csv,
            "eval_mode": args.eval_mode,
            "persistent_planner": args.persistent_planner,
            "iterations": args.max_iters,
            "outer_update_mode": args.outer_update_mode,
            "final_d1": bounded(args.init_d1, args.min_d, args.max_d),
            "final_d2": bounded(args.init_d2, args.min_d, args.max_d),
            "best": None,
        }
        with open(os.path.join(run_dir, "final_result.json"), "w") as f:
            json.dump(final_result, f, indent=2)
        print("FD IRL results written to: {}".format(run_dir), flush=True)
        print("Optimization trace: {}".format(os.path.join(run_dir, "optimization_trace.csv")), flush=True)
        print("Final result: {}".format(os.path.join(run_dir, "final_result.json")), flush=True)
        return 0

    roscore_proc = None
    persistent_evaluator = None
    viz = None
    try:
        roscore_proc = ensure_ros_master(run_dir, args.master_timeout)
        d1 = bounded(args.init_d1, args.min_d, args.max_d)
        d2 = bounded(args.init_d2, args.min_d, args.max_d)
        if not args.disable_visualization:
            ensure_rospy_node("airgrasp_fd_irl_optimizer")
            clear_marker_array_topic("/scene/planned_trajectory_segments", "stale MINCO final/reference")
            viz = IrlVisualization(
                args.marker_topic,
                args.frame_id,
                args.viz_stride,
                scene,
                args.sphere_radius,
                minco_history_limit=args.viz_minco_history_limit,
            )
            viz.set_demo(demo_points)
            try:
                guide_points = guide_points_for_visualization(args, scene)
                if guide_points:
                    viz.set_guide(guide_points)
                    print(
                        "[fd_irl] visualizing S-guide points={}".format(len(guide_points)),
                        flush=True,
                    )
            except Exception as exc:
                print("[fd_irl] warning: failed to visualize s-guide path: {}".format(exc), flush=True)
            viz.set_d_values(d1, d2)
            print("[fd_irl] RViz MarkerArray topic: {}".format(args.marker_topic), flush=True)

        if args.eval_mode == "single_shot" and args.persistent_planner and args.max_iters > 0:
            if args.rho_astar_waypoint_schedule_values:
                print(
                    "[fd_irl] persistent planner uses launch-time rhoAstarWaypoint={:.6f}; "
                    "per-iteration rho schedule is recorded but cannot reconfigure an already-running planner.".format(
                        rho_astar_waypoint_for_iteration(args, 0)
                    ),
                    flush=True,
                )
            persistent_evaluator = PersistentSingleShotEvaluator(args, run_dir, scene, d1, d2)

        eval_rows = []
        trace_rows = []
        best = None

        for iteration in range(args.max_iters):
            rho_astar_waypoint = rho_astar_waypoint_for_iteration(args, iteration)
            if getattr(args, "optimizer_log_mode", "concise") == "verbose":
                print(
                    "[fd_irl] iteration={} rhoAstarWaypoint(stage1)={:.6f} stage2={:.6f}".format(
                        iteration, rho_astar_waypoint, args.rho_astar_waypoint_stage2
                    ),
                    flush=True,
                )
            else:
                print(
                    "[fd_irl] iter {} evaluating 5 cases | d=({:.4f},{:.4f}) eps={:.4f}".format(
                        iteration, d1, d2, args.eps
                    ),
                    flush=True,
                )
            d1_plus, d1_minus, d1_denom = finite_difference_step(d1, args.eps, args.min_d, args.max_d)
            d2_plus, d2_minus, d2_denom = finite_difference_step(d2, args.eps, args.min_d, args.max_d)

            fd_specs = [
                ("d1_plus", d1_plus, d2),
                ("d1_minus", d1_minus, d2),
                ("d2_plus", d1, d2_plus),
                ("d2_minus", d1, d2_minus),
            ]
            use_single_shot_batch = args.eval_mode == "single_shot" and args.s_guide_mode != "adaptive"
            if use_single_shot_batch:
                if viz is not None:
                    viz.set_eval_status(iteration, "batch", d1, d2, "single-shot running")
                batch_specs = [("current", d1, d2)] + fd_specs
                if persistent_evaluator is not None:
                    batch_results = persistent_evaluator.evaluate_batch(
                        args,
                        run_dir,
                        demo_samples,
                        iteration,
                        batch_specs,
                        viz=viz if args.visualize_minco_trajectories else None,
                    )
                else:
                    batch_results = evaluate_parameter_batch(args, run_dir, demo_samples, iteration, batch_specs)
                current = batch_results["current"]
                evals = {label: batch_results[label] for label, _, _ in fd_specs}
                for label, _, _ in batch_specs:
                    result = batch_results[label]
                    result["iteration"] = iteration
                    eval_rows.append(json_safe_eval(result))
                    best = save_best(run_dir, result, best)
                    if (
                        viz is not None and
                        args.visualize_minco_trajectories and
                        persistent_evaluator is None
                    ):
                        visualize_result_minco_trajectories(viz, iteration, result)
                    if viz is not None and label != "current":
                        viz.set_eval_status(iteration, label, result["d1"], result["d2"], result["status"], result["loss"])
            else:
                if viz is not None:
                    viz.set_eval_status(iteration, "current", d1, d2, "running")
                current = evaluate_parameters(args, run_dir, demo_samples, "current", iteration, d1, d2)
                current["iteration"] = iteration
                eval_rows.append(json_safe_eval(current))
                best = save_best(run_dir, current, best)
                if viz is not None and args.visualize_minco_trajectories:
                    visualize_result_minco_trajectories(viz, iteration, current)
                evals = {}
                for label, eval_d1, eval_d2 in fd_specs:
                    if viz is not None:
                        viz.set_eval_status(iteration, label, eval_d1, eval_d2, "running")
                    result = evaluate_parameters(args, run_dir, demo_samples, label, iteration, eval_d1, eval_d2)
                    result["iteration"] = iteration
                    evals[label] = result
                    eval_rows.append(json_safe_eval(result))
                    best = save_best(run_dir, result, best)
                    if viz is not None and args.visualize_minco_trajectories:
                        visualize_result_minco_trajectories(viz, iteration, result)
                    if viz is not None:
                        viz.set_eval_status(iteration, label, eval_d1, eval_d2, result["status"], result["loss"])

            if viz is not None:
                viz.add_iteration(
                    iteration,
                    d1,
                    d2,
                    current["loss"],
                    current["points"],
                    current["status"],
                    "current done\nfinite differences done",
                )

            g1_info = finite_difference_gradient(
                "d1", evals["d1_plus"], evals["d1_minus"], d1_plus, d1_minus, d1_denom
            )
            g2_info = finite_difference_gradient(
                "d2", evals["d2_plus"], evals["d2_minus"], d2_plus, d2_minus, d2_denom
            )
            g1 = g1_info["gradient"]
            g2 = g2_info["gradient"]
            if args.outer_update_mode == "best_neighbor":
                update_info = best_neighbor_update(
                    current,
                    [
                        ("d1_plus", evals["d1_plus"]),
                        ("d1_minus", evals["d1_minus"]),
                        ("d2_plus", evals["d2_plus"]),
                        ("d2_minus", evals["d2_minus"]),
                    ],
                    min_improvement=args.best_neighbor_min_improvement,
                )
                next_d1 = bounded(update_info["next_d1"], args.min_d, args.max_d)
                next_d2 = bounded(update_info["next_d2"], args.min_d, args.max_d)
                step_d1 = next_d1 - d1
                step_d2 = next_d2 - d2
            else:
                next_d1, step_d1 = clipped_update(d1, g1, args.learning_rate, args.max_step, args.min_d, args.max_d)
                next_d2, step_d2 = clipped_update(d2, g2, args.learning_rate, args.max_step, args.min_d, args.max_d)
                update_info = {
                    "accepted_label": "gradient_step",
                    "accepted_loss": "",
                    "accepted_channel": "gradient",
                    "accepted_reason": "finite_difference_gradient",
                }

            trace_row = {
                "iteration": iteration,
                "d1": d1,
                "d2": d2,
                "loss": current["loss"],
                "loss_mode": current["loss_mode"],
                "imitation_loss": current["imitation_loss"],
                "position_loss": current["position_loss"],
                "yaw_loss": current["yaw_loss"],
                "feasibility_loss": current["feasibility_loss"],
                "feasibility_margin_available": current["feasibility_margin_available"],
                "feasibility_margin_source": current["feasibility_margin_source"],
                "feasibility_min_clearance_1": current["feasibility_min_clearance_1"],
                "feasibility_min_clearance_2": current["feasibility_min_clearance_2"],
                "feasibility_min_margin_1": current["feasibility_min_margin_1"],
                "feasibility_min_margin_2": current["feasibility_min_margin_2"],
                "feasibility_violation_1": current["feasibility_violation_1"],
                "feasibility_violation_2": current["feasibility_violation_2"],
                "failure_status_penalty_applied": current["failure_status_penalty_applied"],
                "g1": g1,
                "g2": g2,
                "g1_mode": g1_info["mode"],
                "g2_mode": g2_info["mode"],
                "d1_plus_usable": g1_info["plus_usable"],
                "d1_minus_usable": g1_info["minus_usable"],
                "d2_plus_usable": g2_info["plus_usable"],
                "d2_minus_usable": g2_info["minus_usable"],
                "d1_failed_boundary": g1_info["failed_boundary"],
                "d2_failed_boundary": g2_info["failed_boundary"],
                "outer_update_mode": args.outer_update_mode,
                "accepted_label": update_info["accepted_label"],
                "accepted_loss": update_info["accepted_loss"],
                "accepted_channel": update_info["accepted_channel"],
                "accepted_reason": update_info["accepted_reason"],
                "next_d1": next_d1,
                "next_d2": next_d2,
                "step_d1": step_d1,
                "step_d2": step_d2,
                "status": current["status"],
                "completed_waypoints": current["completed_waypoints"],
                "waypoint_count": current["waypoint_count"],
                "failure_count": current["failure_count"],
                "executed_path_length": current["executed_path_length"],
                "executed_min_clearance_1": current["executed_min_clearance_1"],
                "executed_min_clearance_2": current["executed_min_clearance_2"],
                "executed_min_margin_1": current["executed_min_margin_1"],
                "executed_min_margin_2": current["executed_min_margin_2"],
                "candidate_path_length": current["candidate_path_length"],
                "candidate_min_clearance_1": current["candidate_min_clearance_1"],
                "candidate_min_clearance_2": current["candidate_min_clearance_2"],
                "candidate_min_margin_1": current["candidate_min_margin_1"],
                "candidate_min_margin_2": current["candidate_min_margin_2"],
                "run_dir": current["run_dir"],
                "eval_run_dir": current["eval_run_dir"],
                "trajectory_csv": current["trajectory_csv"],
                "eval_mode": current["eval_mode"],
                "candidate_source": current["candidate_source"],
                "source_trajectory_csv": current["source_trajectory_csv"],
                "selected_planned_trajectory_count": current["selected_planned_trajectory_count"],
                "rho_astar_waypoint": current.get("rho_astar_waypoint", rho_astar_waypoint),
                "rho_astar_waypoint_stage2": current.get("rho_astar_waypoint_stage2", args.rho_astar_waypoint_stage2),
                "enable_tail_constraint": current.get("enable_tail_constraint", args.enable_tail_constraint),
                "minco_warm_start_cache": current.get("minco_warm_start_cache", args.minco_warm_start_cache),
                "minco_warm_start_cache_force_reuse": current.get("minco_warm_start_cache_force_reuse", args.minco_warm_start_cache_force_reuse),
                "s_guide_mode": current.get("s_guide_mode", ""),
                "s_guide_csv": current.get("s_guide_csv", ""),
                "s_guide_json": current.get("s_guide_json", ""),
                "s_guide_target_clearance_1": current.get("s_guide_target_clearance_1", ""),
                "s_guide_target_clearance_2": current.get("s_guide_target_clearance_2", ""),
                "s_guide_min_clearance_1": current.get("s_guide_min_clearance_1", ""),
                "s_guide_min_clearance_2": current.get("s_guide_min_clearance_2", ""),
                "s_guide_clearance_floor_1": current.get("s_guide_clearance_floor_1", ""),
                "s_guide_clearance_floor_2": current.get("s_guide_clearance_floor_2", ""),
                "s_guide_demo_clearance_1": current.get("s_guide_demo_clearance_1", ""),
                "s_guide_demo_clearance_2": current.get("s_guide_demo_clearance_2", ""),
                "s_guide_demo_floor_margin": current.get("s_guide_demo_floor_margin", ""),
                "candidate_load_error": current.get("candidate_load_error", ""),
            }
            trace_rows.append(trace_row)
            write_optimizer_outputs(run_dir, eval_rows, trace_rows)

            boundary_text = ""
            if g1_info["failed_boundary"] or g2_info["failed_boundary"]:
                boundary_text = "\nboundary: {} {}".format(
                    g1_info["failed_boundary"],
                    g2_info["failed_boundary"],
                ).rstrip()
            extra = (
                "g1: {:+.4f}   g2: {:+.4f}\n"
                "gmode: {g1_mode}, {g2_mode}\n"
                "accept: {accepted_label} ({accepted_channel})\n"
                "next: {next_d1:.4f}, {next_d2:.4f}{boundary_text}"
            ).format(
                g1,
                g2,
                g1_mode=g1_info["mode"],
                g2_mode=g2_info["mode"],
                accepted_label=update_info["accepted_label"],
                accepted_channel=update_info["accepted_channel"],
                next_d1=next_d1,
                next_d2=next_d2,
                boundary_text=boundary_text,
            )
            if viz is not None:
                viz.set_status(
                    "FD IRL optimizer\n"
                    "iter: {iteration}\n"
                    "d1: {d1:.4f}   d2: {d2:.4f}\n"
                    "loss: {loss:.6f}\n"
                    "status: {status}\n"
                    "{extra}".format(
                        iteration=iteration,
                        d1=d1,
                        d2=d2,
                        loss=current["loss"],
                        status=current["status"],
                        extra=extra,
                    )
                )

            print_iteration_summary(
                args,
                iteration,
                rho_astar_waypoint,
                current,
                evals,
                g1,
                g2,
                g1_info,
                g2_info,
                update_info,
                next_d1,
                next_d2,
            )
            d1, d2 = next_d1, next_d2

        write_optimizer_outputs(run_dir, eval_rows, trace_rows)
        final_result = {
            "run_dir": run_dir,
            "demo_csv": demo_csv,
            "eval_mode": args.eval_mode,
            "iterations": args.max_iters,
            "outer_update_mode": args.outer_update_mode,
            "final_d1": d1,
            "final_d2": d2,
            "best": best,
        }
        with open(os.path.join(run_dir, "final_result.json"), "w") as f:
            json.dump(final_result, f, indent=2)

        if viz is not None:
            viz.publish()
            time.sleep(max(0.0, args.hold_after_sec))

        print("FD IRL results written to: {}".format(run_dir), flush=True)
        print("Optimization trace: {}".format(os.path.join(run_dir, "optimization_trace.csv")), flush=True)
        print("Final result: {}".format(os.path.join(run_dir, "final_result.json")), flush=True)
        return 0
    finally:
        if persistent_evaluator is not None:
            persistent_evaluator.shutdown()
        if roscore_proc is not None:
            roscore_proc.terminate()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
