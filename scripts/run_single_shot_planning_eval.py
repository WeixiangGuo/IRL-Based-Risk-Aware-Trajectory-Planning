#!/usr/bin/python3

import argparse
import copy
import csv
import datetime as _dt
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

def _prepend_source_path():
    pkg_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    src_path = os.path.join(pkg_root, "src")
    if os.path.isdir(src_path) and src_path not in sys.path:
        sys.path.insert(0, src_path)


_prepend_source_path()

import rosgraph
import rospy
import yaml
from airgrasp_minco_irl.adaptive_guide import (
    DEFAULT_CLEARANCE_BUFFER,
    DEFAULT_MAX_CLEARANCE,
    DEFAULT_MIN_CLEARANCE,
    build_adaptive_s_guide,
    guide_clearance_stats,
    write_guide_artifacts,
)
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommandTrajectory, UAMFullState
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_CASES = "0.10,0.10"


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


def script_package_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def parse_cases(text):
    cases = []
    for raw_item in text.split(";"):
        item = raw_item.strip()
        if not item:
            continue
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 2:
            raise ValueError("Bad case '{}', expected d1,d2".format(item))
        cases.append((float(parts[0]), float(parts[1])))
    if not cases:
        raise ValueError("No d1/d2 cases were provided.")
    return cases


def as_float_list(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must be a list of {} values.".format(name, length))
    return [float(v) for v in value]


def load_scene(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    start_position = as_float_list(data.get("start_position"), 3, "start_position")
    goal_position = as_float_list(data.get("goal_position"), 3, "goal_position")
    reference_path = data.get("reference_path", [])
    if not isinstance(reference_path, list) or len(reference_path) < 2:
        reference_path = [start_position, goal_position]

    pillar_radius = float(data.get("pillar_radius", 0.30))
    pillar1_radius = float(data.get("pillar1_radius", pillar_radius))
    pillar2_radius = float(data.get("pillar2_radius", pillar_radius))
    return {
        "frame_id": str(data.get("frame_id", "world")),
        "start_position": start_position,
        "start_yaw": float(data.get("start_yaw", 0.0)),
        "goal_position": goal_position,
        "goal_yaw": float(data.get("goal_yaw", 0.0)),
        "start_theta": as_float_list(data.get("start_theta", [0.6, -0.55, 0.0]), 3, "start_theta"),
        "start_dtheta": as_float_list(data.get("start_dtheta", [0.0, 0.0, 0.0]), 3, "start_dtheta"),
        "goal_theta": as_float_list(data.get("goal_theta", [0.6, -0.55, 0.0]), 3, "goal_theta"),
        "goal_dtheta": as_float_list(data.get("goal_dtheta", [0.0, 0.0, 0.0]), 3, "goal_dtheta"),
        "pillar1_center_xy": as_float_list(data.get("pillar1_center_xy"), 2, "pillar1_center_xy"),
        "pillar2_center_xy": as_float_list(data.get("pillar2_center_xy"), 2, "pillar2_center_xy"),
        "pillar_radius": pillar_radius,
        "pillar1_radius": pillar1_radius,
        "pillar2_radius": pillar2_radius,
        "pillar_z_min": float(data.get("pillar_z_min", 0.0)),
        "pillar_z_max": float(data.get("pillar_z_max", 2.0)),
        "reference_path": [as_float_list(p, 3, "reference_path") for p in reference_path],
    }


def guide_path_to_spec(path):
    return ";".join([
        "{:.9g},{:.9g},{:.9g}".format(float(p[0]), float(p[1]), float(p[2]))
        for p in path
    ])


def master_is_online():
    try:
        rosgraph.Master("/airgrasp_single_shot_probe").getPid()
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


class ManagedProcess:
    def __init__(self, name, args, log_path, stream_debug=False,
                 stream_debug_stride=20):
        self.name = name
        self.args = list(args)
        self.log_path = log_path
        self.log_file = open(log_path, "w")
        self.stream_debug = bool(stream_debug)
        self.stream_debug_stride = max(1, int(stream_debug_stride))
        self.stream_match_count = 0
        self.stream_thread = None
        self.proc = subprocess.Popen(
            self.args,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        if self.stream_debug:
            self.stream_thread = threading.Thread(
                target=self._stream_debug_lines,
                name="{}_debug_log_stream".format(self.name),
                daemon=True,
            )
            self.stream_thread.start()

    def _should_stream_line(self, line):
        if "[TrajOpt][IRL]" in line:
            return True
        if "[MINCO-IRL]" not in line:
            return False

        self.stream_match_count += 1
        if self.stream_match_count == 1:
            return True
        if self.stream_match_count % self.stream_debug_stride == 0:
            return True
        if "stage=final" in line:
            return True
        if "status=fail" in line or "reject" in line:
            return True
        if "self_cross=0" not in line and "self_cross=" in line:
            return True
        return False

    def _stream_debug_lines(self):
        try:
            with open(self.log_path, "r", errors="replace") as log_reader:
                while True:
                    line = log_reader.readline()
                    if line:
                        if self._should_stream_line(line):
                            sys.stdout.write("[{}] {}".format(self.name, line))
                            sys.stdout.flush()
                        continue

                    if self.proc.poll() is not None:
                        break
                    time.sleep(0.1)
        except Exception as exc:
            sys.stdout.write("[{}] debug log stream stopped: {}\n".format(self.name, exc))
            sys.stdout.flush()

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
        if self.stream_thread is not None:
            self.stream_thread.join(timeout=1.0)
        self.log_file.close()


def quat_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_pose(position, yaw):
    pose = Pose()
    pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    pose.orientation = quat_from_yaw(yaw)
    return pose


def make_goal(scene, seq):
    msg = UAMFullState()
    msg.header.stamp = rospy.Time.now()
    msg.header.seq = int(seq)
    msg.header.frame_id = scene["frame_id"]
    msg.pose = make_pose(scene["goal_position"], scene["goal_yaw"])
    msg.ee_pose = make_pose(scene["goal_position"], scene["goal_yaw"])
    msg.theta = list(scene["goal_theta"])
    msg.dtheta = list(scene["goal_dtheta"])
    return msg


def distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0]) +
        (a[1] - b[1]) * (a[1] - b[1]) +
        (a[2] - b[2]) * (a[2] - b[2])
    )


def path_length(points):
    total = 0.0
    for idx in range(1, len(points)):
        total += distance(points[idx - 1], points[idx])
    return total


def cylinder_surface_distance(point, center_xy, radius, z_min, z_max):
    radial_gap = math.hypot(point[0] - center_xy[0], point[1] - center_xy[1]) - radius
    if z_min <= point[2] <= z_max:
        return radial_gap
    z_gap = z_min - point[2] if point[2] < z_min else point[2] - z_max
    if radial_gap > 0.0:
        return math.sqrt(radial_gap * radial_gap + z_gap * z_gap)
    return z_gap


def clearance_stats(points, scene, sphere_radius, d1, d2):
    if not points:
        return {
            "min_clearance_1": None,
            "min_clearance_2": None,
            "min_margin_1": None,
            "min_margin_2": None,
        }

    min_c1 = float("inf")
    min_c2 = float("inf")
    for p in points:
        c1 = cylinder_surface_distance(
            p,
            scene["pillar1_center_xy"],
            scene["pillar1_radius"],
            scene["pillar_z_min"],
            scene["pillar_z_max"],
        ) - sphere_radius
        c2 = cylinder_surface_distance(
            p,
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
        "min_margin_1": min_c1 - d1,
        "min_margin_2": min_c2 - d2,
    }


def trajectory_positions(trajectory):
    if trajectory is None:
        return []
    return [point["position"] for point in trajectory["points"]]


def downsample_points(points, max_points):
    if len(points) <= max_points:
        return list(points)
    step = int(math.ceil(float(len(points)) / float(max_points)))
    sampled = list(points[::step])
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def max_local_turn_deg(points, stride):
    sampled = downsample_points(points, max(3, int(math.ceil(float(len(points)) / max(1, stride)))))
    max_turn = 0.0
    prev_vec = None
    for idx in range(1, len(sampled)):
        dx = sampled[idx][0] - sampled[idx - 1][0]
        dy = sampled[idx][1] - sampled[idx - 1][1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            continue
        vec = (dx / norm, dy / norm)
        if prev_vec is not None:
            dot = max(-1.0, min(1.0, prev_vec[0] * vec[0] + prev_vec[1] * vec[1]))
            turn = math.degrees(math.acos(dot))
            max_turn = max(max_turn, turn)
        prev_vec = vec
    return max_turn


def resample_positions_by_spacing(points, spacing):
    if len(points) < 2:
        return list(points)
    spacing = max(float(spacing), 1e-4)
    sampled = [list(points[0])]
    current = list(points[0])
    accumulated = 0.0
    idx = 1
    while idx < len(points):
        target = points[idx]
        segment = distance(current, target)
        if segment < 1e-9:
            current = list(target)
            idx += 1
            continue
        if accumulated + segment >= spacing:
            ratio = (spacing - accumulated) / segment
            current = [
                current[axis] + ratio * (target[axis] - current[axis])
                for axis in range(3)
            ]
            sampled.append(list(current))
            accumulated = 0.0
        else:
            accumulated += segment
            current = list(target)
            idx += 1
    if distance(sampled[-1], points[-1]) > 1e-6:
        sampled.append(list(points[-1]))
    return sampled


def max_arc_window_turn_deg(points, spacing, half_window_m):
    sampled = resample_positions_by_spacing(points, spacing)
    if len(sampled) < 3:
        return 0.0
    half_window_steps = max(1, int(round(float(half_window_m) / max(float(spacing), 1e-4))))
    max_turn = 0.0
    for idx in range(half_window_steps, len(sampled) - half_window_steps):
        prev_point = sampled[idx - half_window_steps]
        point = sampled[idx]
        next_point = sampled[idx + half_window_steps]
        ax = point[0] - prev_point[0]
        ay = point[1] - prev_point[1]
        bx = next_point[0] - point[0]
        by = next_point[1] - point[1]
        prev_norm = math.hypot(ax, ay)
        next_norm = math.hypot(bx, by)
        if prev_norm < 1e-6 or next_norm < 1e-6:
            continue
        dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (prev_norm * next_norm)))
        max_turn = max(max_turn, math.degrees(math.acos(dot)))
    return max_turn


def _orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a, b, c, d):
    eps = 1e-9

    def on_segment(p, q, r):
        return (
            min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps and
            min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps
        )

    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if o1 * o2 < -eps and o3 * o4 < -eps:
        return True
    if abs(o1) <= eps and on_segment(a, c, b):
        return True
    if abs(o2) <= eps and on_segment(a, d, b):
        return True
    if abs(o3) <= eps and on_segment(c, a, d):
        return True
    if abs(o4) <= eps and on_segment(c, b, d):
        return True
    return False


def self_intersection_count(points, max_points):
    sampled = downsample_points(points, max(4, max_points))
    count = 0
    for i in range(len(sampled) - 1):
        a = sampled[i]
        b = sampled[i + 1]
        if math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6:
            continue
        for j in range(i + 2, len(sampled) - 1):
            if i == 0 and j == len(sampled) - 2:
                continue
            c = sampled[j]
            d = sampled[j + 1]
            if math.hypot(c[0] - d[0], c[1] - d[1]) < 1e-6:
                continue
            if _segments_intersect(a, b, c, d):
                count += 1
    return count


def format_reason(name, actual, threshold):
    return "{} {:.4f} < {:.4f}".format(name, actual, threshold)


def evaluate_trajectory_quality(traj_idx, trajectory, scene, sphere_radius, d1, d2, args):
    points = trajectory_positions(trajectory)
    stats = clearance_stats(points, scene, sphere_radius, d1, d2)
    length = path_length(points)
    chord = distance(points[0], points[-1]) if len(points) >= 2 else 0.0
    length_ratio = length / chord if chord > 1e-6 else float("inf")
    turn_deg = max_local_turn_deg(points, args.quality_sample_stride) if len(points) >= 3 else 0.0
    arc_turn_deg = max_arc_window_turn_deg(
        points,
        args.quality_arc_turn_spacing,
        args.quality_arc_turn_half_window_m,
    ) if len(points) >= 3 else 0.0
    intersections = self_intersection_count(points, args.quality_self_intersection_max_points)

    reasons = []
    if stats["min_clearance_1"] is None:
        reasons.append("empty trajectory")
    else:
        if stats["min_clearance_1"] < args.quality_min_clearance_1:
            reasons.append(format_reason("c1", stats["min_clearance_1"], args.quality_min_clearance_1))
        if stats["min_clearance_2"] < args.quality_min_clearance_2:
            reasons.append(format_reason("c2", stats["min_clearance_2"], args.quality_min_clearance_2))
    if length > args.quality_max_path_length:
        reasons.append("length {:.4f} > {:.4f}".format(length, args.quality_max_path_length))
    if length_ratio > args.quality_max_length_ratio:
        reasons.append("length_ratio {:.4f} > {:.4f}".format(length_ratio, args.quality_max_length_ratio))
    if turn_deg > args.quality_max_local_turn_deg:
        reasons.append("turn {:.4f} > {:.4f}".format(turn_deg, args.quality_max_local_turn_deg))
    if arc_turn_deg > args.quality_max_arc_turn_deg:
        reasons.append("arc_turn {:.4f} > {:.4f}".format(arc_turn_deg, args.quality_max_arc_turn_deg))
    if intersections > args.quality_max_self_intersections:
        reasons.append("self_intersections {} > {}".format(intersections, args.quality_max_self_intersections))

    c1_for_score = stats["min_clearance_1"] if stats["min_clearance_1"] is not None else -10.0
    c2_for_score = stats["min_clearance_2"] if stats["min_clearance_2"] is not None else -10.0
    clearance_penalty = 100.0 * (
        max(0.0, args.quality_min_clearance_1 - c1_for_score) +
        max(0.0, args.quality_min_clearance_2 - c2_for_score)
    )
    length_penalty = 3.0 * max(0.0, length - args.quality_max_path_length)
    ratio_penalty = 30.0 * max(0.0, length_ratio - args.quality_max_length_ratio)
    turn_penalty = 0.5 * max(0.0, turn_deg - args.quality_max_local_turn_deg)
    arc_turn_penalty = 1.0 * max(0.0, arc_turn_deg - args.quality_max_arc_turn_deg)
    cross_penalty = 40.0 * max(0, intersections - args.quality_max_self_intersections)
    # Tie-breakers keep accepted candidates short and smooth without overriding gates.
    score = clearance_penalty + length_penalty + ratio_penalty + turn_penalty + arc_turn_penalty + cross_penalty
    score += 0.002 * length + 0.002 * turn_deg + 0.002 * arc_turn_deg + 0.001 * intersections

    return {
        "trajectory_index": traj_idx,
        "trajectory_id": trajectory["trajectory_id"],
        "point_count": len(points),
        "path_length": length,
        "chord_length": chord,
        "length_ratio": length_ratio,
        "min_clearance_1": stats["min_clearance_1"],
        "min_clearance_2": stats["min_clearance_2"],
        "min_margin_1": stats["min_margin_1"],
        "min_margin_2": stats["min_margin_2"],
        "max_local_turn_deg": turn_deg,
        "max_arc_turn_deg": arc_turn_deg,
        "self_intersection_count": intersections,
        "quality_pass": len(reasons) == 0,
        "quality_score": score,
        "quality_reasons": "|".join(reasons),
    }


def evaluate_trajectory_qualities(trajectories, scene, sphere_radius, d1, d2, args):
    return [
        evaluate_trajectory_quality(idx, traj, scene, sphere_radius, d1, d2, args)
        for idx, traj in enumerate(trajectories)
    ]


def select_trajectory(trajectories, quality_rows, mode):
    if not trajectories:
        return None, {
            "selection_mode": mode,
            "selected_trajectory_index": "",
            "selected_trajectory_id": "",
            "quality_pass": False,
            "quality_score": "",
            "quality_reasons": "no trajectory",
        }

    if mode == "first":
        selected_idx = 0
    else:
        passing = [row for row in quality_rows if row["quality_pass"]]
        ranked = passing if passing else quality_rows
        selected_idx = int(min(ranked, key=lambda row: row["quality_score"])["trajectory_index"])

    row = quality_rows[selected_idx] if quality_rows else {}
    return trajectories[selected_idx], {
        "selection_mode": mode,
        "selected_trajectory_index": selected_idx,
        "selected_trajectory_id": trajectories[selected_idx]["trajectory_id"],
        "quality_pass": bool(row.get("quality_pass", True)),
        "quality_score": row.get("quality_score", ""),
        "quality_reasons": row.get("quality_reasons", ""),
        "max_local_turn_deg": row.get("max_local_turn_deg", ""),
        "max_arc_turn_deg": row.get("max_arc_turn_deg", ""),
        "length_ratio": row.get("length_ratio", ""),
        "self_intersection_count": row.get("self_intersection_count", ""),
    }


def quality_required(mode):
    return mode == "best_quality"


class SingleShotClient:
    def __init__(self, scene, publish_hz, odom_source="fixed", odom_topic="/drone0/odom",
                 joint_state_topic="/joint_state_est_sim"):
        self.scene = scene
        self.publish_hz = max(float(publish_hz), 1.0)
        self.odom_source = odom_source
        self.odom_topic = odom_topic
        self.joint_state_topic = joint_state_topic
        self.lock = threading.Lock()
        self.active = False
        self.case_start_wall = 0.0
        self.latest_odom = None
        self.trajectories = []
        self.failures = []
        self._stop = False
        self.publisher_thread = None

        self.hold_pub = rospy.Publisher("/scene/planner_hold", Bool, queue_size=1, latch=True)
        self.odom_pub = None
        self.joint_pub = None
        if self.odom_source == "fixed":
            self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
            self.joint_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=10)
        self.goal_pub = rospy.Publisher("/drone0/planning/uam_state_goal_cmd", UAMFullState, queue_size=10)
        self.goal_vis_pub = rospy.Publisher("/drone0/planning/uam_state_goal_vis", UAMFullState, queue_size=10)
        self.goal_vis_mirror_pub = rospy.Publisher("/px4ctrl/uam_state_goal_vis", UAMFullState, queue_size=10)
        self.selected_traj_pub = rospy.Publisher(
            "/irl/selected_position_command_trajectory",
            PositionCommandTrajectory,
            queue_size=1,
            latch=True,
        )
        self.safety_pub = rospy.Publisher(
            "/drone0/planning/irl_safety_distances", Vector3, queue_size=1, latch=True
        )

        self.traj_sub = rospy.Subscriber(
            "/drone0/planning/planned_position_command_trajectory",
            PositionCommandTrajectory,
            self._traj_cb,
            queue_size=50,
        )
        self.failure_sub = rospy.Subscriber(
            "/drone0/planning/planner_failure", Header, self._failure_cb, queue_size=100
        )
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=20)
        if self.odom_source == "fixed":
            self.publisher_thread = threading.Thread(target=self._fixed_state_loop)
            self.publisher_thread.daemon = True
            self.publisher_thread.start()

    def shutdown(self):
        self._stop = True
        if self.publisher_thread is not None:
            self.publisher_thread.join(timeout=1.0)

    def _odom_cb(self, msg):
        with self.lock:
            self.latest_odom = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]

    def _traj_cb(self, msg):
        if len(msg.points) < 2:
            return
        points = []
        header_stamp = msg.header.stamp
        for idx, pt in enumerate(msg.points):
            t_rel = (pt.header.stamp - header_stamp).to_sec()
            if t_rel < 0.0:
                t_rel = idx * msg.sample_dt if msg.sample_dt > 0.0 else 0.0
            points.append({
                "idx": idx,
                "t": t_rel,
                "position": [pt.position.x, pt.position.y, pt.position.z],
                "yaw": pt.yaw,
            })
        item = {
            "stamp": msg.header.stamp.to_sec(),
            "wall": time.time(),
            "trajectory_id": int(msg.trajectory_id),
            "sample_dt": float(msg.sample_dt),
            "total_duration": float(msg.total_duration),
            "points": points,
            "msg": copy.deepcopy(msg),
        }
        with self.lock:
            if self.active and item["wall"] >= self.case_start_wall:
                self.trajectories.append(item)

    def _failure_cb(self, msg):
        item = {
            "stamp": msg.stamp.to_sec(),
            "wall": time.time(),
            "seq": int(msg.seq),
            "reason": msg.frame_id,
        }
        with self.lock:
            if self.active and item["wall"] >= self.case_start_wall:
                self.failures.append(item)

    def _fixed_state_loop(self):
        rate = rospy.Rate(self.publish_hz)
        while not rospy.is_shutdown() and not self._stop:
            self.publish_fixed_state()
            rate.sleep()

    def publish_fixed_state(self):
        if self.odom_source != "fixed":
            return
        odom = Odometry()
        odom.header = Header(stamp=rospy.Time.now(), frame_id=self.scene["frame_id"])
        odom.child_frame_id = "base_link"
        odom.pose.pose = make_pose(self.scene["start_position"], self.scene["start_yaw"])
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0
        self.odom_pub.publish(odom)

        joint = JointState()
        joint.header.stamp = rospy.Time.now()
        joint.name = ["arm_joint_0", "arm_joint_1", "arm_joint_2"]
        joint.position = list(self.scene["start_theta"])
        joint.velocity = list(self.scene["start_dtheta"])
        joint.effort = [0.0 for _ in joint.position]
        self.joint_pub.publish(joint)

    def publish_hold(self, hold):
        self.hold_pub.publish(Bool(data=bool(hold)))

    def publish_safety(self, d1, d2, sphere_radius, repeats=5, sleep_sec=0.1):
        msg = Vector3(x=float(d1), y=float(d2), z=float(sphere_radius))
        for _ in range(max(1, repeats)):
            self.safety_pub.publish(msg)
            rospy.sleep(sleep_sec)

    def publish_goal(self, seq):
        msg = make_goal(self.scene, seq)
        self.goal_pub.publish(msg)
        self.goal_vis_pub.publish(msg)
        self.goal_vis_mirror_pub.publish(msg)
        return msg.header.stamp.to_sec()

    def publish_selected_trajectory(self, trajectory, repeats=3, sleep_sec=0.05):
        if trajectory is None:
            return False
        msg = trajectory.get("msg")
        if msg is None:
            return False
        for _ in range(max(1, repeats)):
            self.selected_traj_pub.publish(msg)
            rospy.sleep(sleep_sec)
        return True

    def wait_for_connections(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            has_state_connection = True
            if self.odom_source == "fixed":
                has_state_connection = (
                    self.odom_pub.get_num_connections() > 0 and
                    self.joint_pub.get_num_connections() > 0
                )
            if has_state_connection and self.goal_pub.get_num_connections() > 0 and self.safety_pub.get_num_connections() > 0:
                return True
            rospy.sleep(0.1)
        return False

    def wait_for_odom_at_start(self, tolerance, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                pos = list(self.latest_odom) if self.latest_odom is not None else None
            if pos is not None and distance(pos, self.scene["start_position"]) <= tolerance:
                return True
            rospy.sleep(0.1)
        return False

    def wait_for_any_odom(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.latest_odom is not None:
                    return True
            rospy.sleep(0.1)
        return False

    def begin_case(self):
        with self.lock:
            self.active = True
            self.case_start_wall = time.time()
            self.trajectories = []
            self.failures = []

    def end_case(self):
        with self.lock:
            self.active = False

    def wait_for_first_trajectory(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.trajectories:
                    return self.trajectories[0]
            rospy.sleep(0.05)
        return None

    def snapshot(self):
        with self.lock:
            return {
                "trajectories": list(self.trajectories),
                "failures": list(self.failures),
            }


def start_processes(args, run_dir, scene, s_guide_path, cases):
    pkg_root = script_package_root()
    world_launch = args.world_launch or os.path.join(pkg_root, "launch", "two_pillar_world.launch")
    planning_launch = args.planning_launch or os.path.join(pkg_root, "launch", "two_pillar_planning.launch")
    initial_d1, initial_d2 = cases[0] if cases else (0.0, 0.0)

    subprocess.run(["rosparam", "load", args.scene_config], check=True)
    processes = []
    if not args.skip_world_launch:
        processes.append(
            ManagedProcess(
            "world",
            [
                "roslaunch",
                world_launch,
                "scene_config:={}".format(os.path.abspath(args.scene_config)),
                "publish_point_clouds:=false",
                "sphere_radius:={:.6f}".format(args.sphere_radius),
                "initial_d1:={:.6f}".format(initial_d1),
                "initial_d2:={:.6f}".format(initial_d2),
                "safety_zone_buffer:={:.6f}".format(args.safety_zone_buffer),
            ],
            os.path.join(run_dir, "world.log"),
            )
        )
    processes.append(
        ManagedProcess(
            "planning",
            [
                "roslaunch",
                planning_launch,
                "obstacles_inflation:={:.3f}".format(args.obstacles_inflation),
                "rho_astar_waypoint:={:.6f}".format(args.rho_astar_waypoint),
                "rho_astar_waypoint_stage2:={:.6f}".format(args.rho_astar_waypoint_stage2),
                "enable_tail_constraint:={}".format(str(args.enable_tail_constraint).lower()),
                "minco_warm_start_cache:={}".format(str(args.minco_warm_start_cache).lower()),
                "minco_warm_start_cache_force_reuse:={}".format(str(args.minco_warm_start_cache_force_reuse).lower()),
                "irl_collision_weight_scale:={:.6f}".format(args.irl_collision_weight_scale),
                "irl_obs_optimization_margin:={:.6f}".format(args.irl_obs_optimization_margin),
                "hide_native_planning_traj:={}".format(str(args.hide_native_planning_traj).lower()),
                "joint_state_est_topic:={}".format(args.joint_state_topic),
                "position_cmd_topic:={}".format(args.position_cmd_topic),
                "irl_cylinder_1_x:={:.6f}".format(scene["pillar1_center_xy"][0]),
                "irl_cylinder_1_y:={:.6f}".format(scene["pillar1_center_xy"][1]),
                "irl_cylinder_2_x:={:.6f}".format(scene["pillar2_center_xy"][0]),
                "irl_cylinder_2_y:={:.6f}".format(scene["pillar2_center_xy"][1]),
                "irl_cylinder_radius:={:.6f}".format(scene["pillar_radius"]),
                "irl_cylinder_1_radius:={:.6f}".format(scene["pillar1_radius"]),
                "irl_cylinder_2_radius:={:.6f}".format(scene["pillar2_radius"]),
                "irl_cylinder_z_min:={:.6f}".format(scene["pillar_z_min"]),
                "irl_cylinder_z_max:={:.6f}".format(scene["pillar_z_max"]),
                "s_guide_path_enable:={}".format(str(args.s_guide_enable).lower()),
                "s_guide_bypass_astar:={}".format(str(args.s_guide_bypass_astar).lower()),
                "s_guide_endpoint_tolerance:={:.6f}".format(args.s_guide_endpoint_tolerance),
                "s_guide_path:={}".format(s_guide_path),
            ],
            os.path.join(run_dir, "planning.log"),
            stream_debug=args.stream_planning_debug,
            stream_debug_stride=args.stream_planning_debug_stride,
        )
    )
    if args.use_minco_final_trajectory_visualizer:
        processes.append(
            ManagedProcess(
                "minco_final_visualizer",
                [
                    "rosrun",
                    "airgrasp_minco_irl",
                    "minco_final_trajectory_visualizer.py",
                    "__name:=minco_final_trajectory_visualizer_eval",
                    "_frame_id:={}".format(scene["frame_id"]),
                    "_traj_topic:=/drone0/planning/planned_position_command_trajectory",
                    "_marker_topic:=/scene/planned_trajectory_segments",
                    "_sample_stride:=1",
                    "_line_width:={:.6f}".format(args.minco_final_line_width),
                    "_reference_csv:={}".format(args.minco_reference_csv),
                    "_reference_line_width:={:.6f}".format(args.minco_reference_line_width),
                    "_alpha:=1.0",
                    "_min_points:=20",
                    "_min_path_length:=0.10",
                    "_show_direction_ticks:=false",
                ],
                os.path.join(run_dir, "minco_final_visualizer.log"),
            )
        )
    return processes


def stop_processes(processes):
    for proc in reversed(processes):
        proc.terminate()


def wait_for_map(timeout):
    try:
        rospy.wait_for_message("/sdf_map/esdf", PointCloud2, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def publish_delete_all_marker(topic, label):
    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.2)
    marker = Marker()
    marker.action = Marker.DELETEALL
    pub.publish(MarkerArray(markers=[marker]))
    rospy.loginfo("[single_shot] cleared %s markers on %s.", label, topic)


def publish_delete_marker(topic, label):
    pub = rospy.Publisher(topic, Marker, queue_size=1, latch=True)
    rospy.sleep(0.1)
    marker = Marker()
    marker.action = Marker.DELETEALL
    pub.publish(marker)
    rospy.loginfo("[single_shot] cleared %s marker on %s.", label, topic)


def publish_empty_path(topic, label, frame_id):
    pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
    rospy.sleep(0.1)
    msg = Path()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    pub.publish(msg)
    rospy.loginfo("[single_shot] cleared %s path on %s.", label, topic)


def publish_empty_pointcloud(topic, label, frame_id):
    pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    rospy.sleep(0.1)
    msg = PointCloud2()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    msg.height = 1
    msg.width = 0
    msg.is_dense = True
    pub.publish(msg)
    rospy.loginfo("[single_shot] cleared %s point cloud on %s.", label, topic)


def clear_legacy_scene_waypoint_markers():
    publish_delete_all_marker(
        "/scene/two_pillar_start_goal_markers",
        "legacy two-pillar scene",
    )
    publish_delete_marker("/visualization/applied_trajectory", "legacy eight applied trajectory")
    publish_delete_marker("/visualization/waypoints", "legacy eight waypoints")
    publish_delete_marker("/visualization/route", "legacy eight route")


def clear_optimizer_markers():
    publish_delete_all_marker(
        "/scene/irl_fd_optimizer_markers",
        "stale FD optimizer",
    )


def clear_native_planning_visuals(frame_id):
    publish_empty_path("/drone0/planning/traj", "native planner traj", frame_id)
    publish_empty_pointcloud("/drone0/planning/traj_wayPts", "native planner traj waypoints", frame_id)
    publish_delete_all_marker("/drone0/planning/traj_yaws", "native planner traj yaws")
    publish_delete_all_marker("/scene/planned_trajectory_segments", "stale MINCO final/reference")


def write_trajectory_csv(path, trajectory):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["stamp", "elapsed", "x", "y", "z", "yaw"])
        writer.writeheader()
        stamp = trajectory["stamp"]
        for point in trajectory["points"]:
            writer.writerow({
                "stamp": "{:.9f}".format(stamp + point["t"]),
                "elapsed": "{:.9f}".format(point["t"]),
                "x": "{:.9f}".format(point["position"][0]),
                "y": "{:.9f}".format(point["position"][1]),
                "z": "{:.9f}".format(point["position"][2]),
                "yaw": "{:.9f}".format(point["yaw"]),
            })


def write_planned_trajectories_csv(path, trajectories):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trajectory_index",
                "trajectory_id",
                "trajectory_stamp",
                "point_index",
                "t",
                "x",
                "y",
                "z",
                "yaw",
            ],
        )
        writer.writeheader()
        for traj_idx, traj in enumerate(trajectories):
            for point in traj["points"]:
                writer.writerow({
                    "trajectory_index": traj_idx,
                    "trajectory_id": traj["trajectory_id"],
                    "trajectory_stamp": "{:.9f}".format(traj["stamp"]),
                    "point_index": point["idx"],
                    "t": "{:.9f}".format(point["t"]),
                    "x": "{:.9f}".format(point["position"][0]),
                    "y": "{:.9f}".format(point["position"][1]),
                    "z": "{:.9f}".format(point["position"][2]),
                    "yaw": "{:.9f}".format(point["yaw"]),
                })


def summarize_case(case_idx, d1, d2, sphere_radius, scene, trajectory, snapshot,
                   duration_sec, case_dir, map_ready, selection_meta, args):
    candidate_points = []
    if trajectory is not None:
        candidate_points = [point["position"] for point in trajectory["points"]]
    stats = clearance_stats(candidate_points, scene, sphere_radius, d1, d2)
    failure_reasons = "|".join([f["reason"] for f in snapshot["failures"]])
    accepted = trajectory is not None
    if accepted and quality_required(selection_meta.get("selection_mode")):
        accepted = bool(selection_meta.get("quality_pass", False))
    if trajectory is None:
        status = "failed"
    elif accepted:
        status = "success"
    else:
        status = "failed_quality_gate"
    return {
        "case_id": case_idx,
        "d1": d1,
        "d2": d2,
        "sphere_radius": sphere_radius,
        "safety_zone_buffer": args.safety_zone_buffer,
        "pillar1_radius": scene["pillar1_radius"],
        "pillar2_radius": scene["pillar2_radius"],
        "analytic_safety_radius_1": scene["pillar1_radius"] + sphere_radius + d1 + args.safety_zone_buffer,
        "analytic_safety_radius_2": scene["pillar2_radius"] + sphere_radius + d2 + args.safety_zone_buffer,
        "rho_astar_waypoint": rospy.get_param("/drone0/planning/rhoAstarWaypoint", ""),
        "rho_astar_waypoint_stage2": rospy.get_param("/drone0/planning/rhoAstarWaypointStage2", ""),
        "enable_tail_constraint": rospy.get_param("/drone0/planning/enable_tail_constraint", ""),
        "minco_warm_start_cache": rospy.get_param("/drone0/planning/mincoWarmStartCacheEnable", ""),
        "minco_warm_start_cache_force_reuse": rospy.get_param("/drone0/planning/mincoWarmStartCacheForceReuse", ""),
        "irl_collision_weight_scale": rospy.get_param("/drone0/planning/irl_collision_weight_scale", ""),
        "irl_obs_optimization_margin": rospy.get_param("/drone0/planning/irl_obs_optimization_margin", ""),
        "status": status,
        "completed_waypoints": 1 if accepted else 0,
        "waypoint_count": 1,
        "duration_sec": duration_sec,
        "failure_count": len(snapshot["failures"]),
        "failure_reasons": failure_reasons,
        "planned_traj_count": len(snapshot["trajectories"]),
        "planned_point_count": len(candidate_points),
        "candidate_point_count": len(candidate_points),
        "candidate_path_length": path_length(candidate_points),
        "candidate_min_clearance_1": stats["min_clearance_1"],
        "candidate_min_clearance_2": stats["min_clearance_2"],
        "candidate_min_margin_1": stats["min_margin_1"],
        "candidate_min_margin_2": stats["min_margin_2"],
        "planned_min_clearance_1": stats["min_clearance_1"],
        "planned_min_clearance_2": stats["min_clearance_2"],
        "planned_min_margin_1": stats["min_margin_1"],
        "planned_min_margin_2": stats["min_margin_2"],
        "executed_path_length": "",
        "executed_min_clearance_1": "",
        "executed_min_clearance_2": "",
        "executed_min_margin_1": "",
        "executed_min_margin_2": "",
        "map_ready": bool(map_ready),
        "trajectory_selection": selection_meta.get("selection_mode", ""),
        "selected_trajectory_index": selection_meta.get("selected_trajectory_index", ""),
        "selected_trajectory_id": selection_meta.get("selected_trajectory_id", ""),
        "trajectory_quality_pass": selection_meta.get("quality_pass", ""),
        "trajectory_quality_score": selection_meta.get("quality_score", ""),
        "trajectory_quality_reasons": selection_meta.get("quality_reasons", ""),
        "candidate_max_local_turn_deg": selection_meta.get("max_local_turn_deg", ""),
        "candidate_max_arc_turn_deg": selection_meta.get("max_arc_turn_deg", ""),
        "candidate_length_ratio": selection_meta.get("length_ratio", ""),
        "candidate_self_intersection_count": selection_meta.get("self_intersection_count", ""),
        "candidate_quality_csv": os.path.abspath(os.path.join(case_dir, "candidate_quality.csv")),
        "candidate_trajectory_csv": os.path.abspath(os.path.join(case_dir, "candidate_planned_path.csv")),
        "source_trajectory_csv": os.path.abspath(os.path.join(case_dir, "planned_trajectories.csv")),
        "case_dir": os.path.abspath(case_dir),
    }


def write_quality_csv(path, quality_rows):
    fields = [
        "trajectory_index",
        "trajectory_id",
        "point_count",
        "path_length",
        "chord_length",
        "length_ratio",
        "min_clearance_1",
        "min_clearance_2",
        "min_margin_1",
        "min_margin_2",
        "max_local_turn_deg",
        "max_arc_turn_deg",
        "self_intersection_count",
        "quality_pass",
        "quality_score",
        "quality_reasons",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in quality_rows:
            writer.writerow(row)


def write_case_outputs(case_dir, case_meta, trajectory, snapshot, quality_rows, selection_meta):
    planned_csv = os.path.join(case_dir, "planned_trajectories.csv")
    write_planned_trajectories_csv(planned_csv, snapshot["trajectories"])
    write_quality_csv(os.path.join(case_dir, "candidate_quality.csv"), quality_rows)
    candidate_csv = os.path.join(case_dir, "candidate_planned_path.csv")
    if trajectory is not None:
        write_trajectory_csv(candidate_csv, trajectory)
    else:
        with open(candidate_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["stamp", "elapsed", "x", "y", "z", "yaw"])
            writer.writeheader()
    with open(os.path.join(case_dir, "case.json"), "w") as f:
        json.dump({
            "case": case_meta,
            "failures": snapshot["failures"],
            "planned_trajectories": [
                {
                    "trajectory_id": traj["trajectory_id"],
                    "stamp": traj["stamp"],
                    "sample_dt": traj["sample_dt"],
                    "total_duration": traj["total_duration"],
                    "point_count": len(traj["points"]),
                }
                for traj in snapshot["trajectories"]
            ],
            "trajectory_selection": selection_meta,
            "quality_rows": quality_rows,
        }, f, indent=2)


def write_summary(run_dir, rows):
    fields = [
        "case_id",
        "d1",
        "d2",
        "sphere_radius",
        "safety_zone_buffer",
        "pillar1_radius",
        "pillar2_radius",
        "analytic_safety_radius_1",
        "analytic_safety_radius_2",
        "rho_astar_waypoint",
        "rho_astar_waypoint_stage2",
        "enable_tail_constraint",
        "minco_warm_start_cache",
        "minco_warm_start_cache_force_reuse",
        "irl_collision_weight_scale",
        "irl_obs_optimization_margin",
        "status",
        "completed_waypoints",
        "waypoint_count",
        "duration_sec",
        "failure_count",
        "failure_reasons",
        "planned_traj_count",
        "planned_point_count",
        "candidate_point_count",
        "candidate_path_length",
        "candidate_min_clearance_1",
        "candidate_min_clearance_2",
        "candidate_min_margin_1",
        "candidate_min_margin_2",
        "planned_min_clearance_1",
        "planned_min_clearance_2",
        "planned_min_margin_1",
        "planned_min_margin_2",
        "executed_path_length",
        "executed_min_clearance_1",
        "executed_min_clearance_2",
        "executed_min_margin_1",
        "executed_min_margin_2",
        "map_ready",
        "trajectory_selection",
        "selected_trajectory_index",
        "selected_trajectory_id",
        "trajectory_quality_pass",
        "trajectory_quality_score",
        "trajectory_quality_reasons",
        "candidate_max_local_turn_deg",
        "candidate_max_arc_turn_deg",
        "candidate_length_ratio",
        "candidate_self_intersection_count",
        "candidate_quality_csv",
        "candidate_trajectory_csv",
        "source_trajectory_csv",
        "case_dir",
    ]
    csv_path = os.path.join(run_dir, "summary.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    json_path = os.path.join(run_dir, "summary.json")
    with open(json_path, "w") as f:
        json.dump(rows, f, indent=2)
    return csv_path, json_path


def build_arg_parser():
    pkg_root = script_package_root()
    parser = argparse.ArgumentParser(
        description="Run single-shot start-to-goal planning evaluations for two-pillar d1/d2 IRL."
    )
    parser.add_argument("--cases", default=DEFAULT_CASES,
                        help="Semicolon-separated d1,d2 pairs. Default: {}".format(DEFAULT_CASES))
    parser.add_argument("--sphere-radius", type=float, default=0.25)
    parser.add_argument("--scene-config", default=os.path.join(pkg_root, "config", "two_pillar_scene.yaml"))
    parser.add_argument("--results-dir", default=os.path.join(pkg_root, "results", "scratch"))
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--obstacles-inflation", type=float, default=0.40)
    parser.add_argument("--safety-zone-buffer", type=float, default=0.0,
                        help="Extra radius added to pillar_radius + sphere_radius + d_j for safety visualization and analytic checks.")
    parser.add_argument("--rho-astar-waypoint", type=float, default=0.0)
    parser.add_argument("--rho-astar-waypoint-stage2", type=float, default=0.0)
    parser.add_argument("--enable-tail-constraint", type=parse_bool, default=False,
                        help="IRL experiment override for planning/enable_tail_constraint.")
    parser.add_argument("--minco-warm-start-cache", type=parse_bool, default=False)
    parser.add_argument("--minco-warm-start-cache-force-reuse", type=parse_bool, default=False,
                        help="Reuse the first stored MINCO stage-1 solution even when tiny guide/state differences appear between cases.")
    parser.add_argument("--irl-collision-weight-scale", type=float, default=3.0,
                        help="ROS override for planning/irl_collision_weight_scale; scales only the IRL obstacle cost.")
    parser.add_argument("--irl-obs-optimization-margin", type=float, default=0.05,
                        help="Extra clearance used only by the MINCO IRL obstacle cost; hard checks still use raw d1/d2.")
    parser.add_argument("--hide-native-planning-traj", type=parse_bool, default=True,
                        help="Hide the planner's native blue /drone0/planning/traj Path in IRL runs.")
    parser.add_argument("--master-timeout", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=35.0)
    parser.add_argument("--map-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=35.0)
    parser.add_argument("--reset-settle-sec", type=float, default=1.0)
    parser.add_argument("--case-settle-sec", type=float, default=0.5)
    parser.add_argument("--cleanup-sec", type=float, default=2.0)
    parser.add_argument("--odom-source", choices=["fixed", "external"], default="fixed",
                        help="fixed publishes the scene start odom; external only subscribes to the real odom topic.")
    parser.add_argument("--odom-topic", default="/drone0/odom")
    parser.add_argument("--joint-state-topic", default="/joint_state_est_sim")
    parser.add_argument("--position-cmd-topic", default="/position_cmd",
                        help="Planner live PositionCommand output. Use a private topic when a safety gate will publish /position_cmd.")
    parser.add_argument("--skip-world-launch", type=parse_bool, default=False,
                        help="Do not launch the IRL world/map process; use already-running real-world/map nodes.")
    parser.add_argument("--require-start-near", type=parse_bool, default=False,
                        help="Fail instead of warn if current odom is not near scene start_position.")
    parser.add_argument("--fixed-odom-hz", type=float, default=50.0)
    parser.add_argument("--start-tolerance", type=float, default=0.05)
    parser.add_argument("--s-guide-enable", type=parse_bool, default=True)
    parser.add_argument("--s-guide-bypass-astar", type=parse_bool, default=True)
    parser.add_argument("--s-guide-endpoint-tolerance", type=float, default=0.45)
    parser.add_argument("--s-guide-mode", choices=["fixed", "adaptive"], default="fixed",
                        help="fixed uses --s-guide-path/reference path; adaptive generates a d-conditioned guide for one case.")
    parser.add_argument("--s-guide-path", default=None,
                        help="Semicolon-separated x,y,z points. Default: reference_path/waypoints from scene config.")
    parser.add_argument("--s-guide-clearance-buffer", type=float, default=DEFAULT_CLEARANCE_BUFFER)
    parser.add_argument("--s-guide-min-clearance", type=float, default=DEFAULT_MIN_CLEARANCE)
    parser.add_argument("--s-guide-max-clearance", type=float, default=DEFAULT_MAX_CLEARANCE)
    parser.add_argument("--trajectory-selection", choices=["first", "best_quality"], default="first",
                        help="first keeps the previous behavior; best_quality collects candidates briefly and selects the best one passing quality gates.")
    parser.add_argument("--trajectory-collection-sec", type=float, default=2.0,
                        help="Extra collection time after the first trajectory when --trajectory-selection=best_quality.")
    parser.add_argument("--quality-min-clearance-1", type=float, default=0.60)
    parser.add_argument("--quality-min-clearance-2", type=float, default=0.28)
    parser.add_argument("--quality-max-path-length", type=float, default=12.5)
    parser.add_argument("--quality-max-length-ratio", type=float, default=1.65)
    parser.add_argument("--quality-max-local-turn-deg", type=float, default=25.0)
    parser.add_argument("--quality-max-arc-turn-deg", type=float, default=35.0)
    parser.add_argument("--quality-arc-turn-spacing", type=float, default=0.05)
    parser.add_argument("--quality-arc-turn-half-window-m", type=float, default=0.10)
    parser.add_argument("--quality-max-self-intersections", type=int, default=0)
    parser.add_argument("--quality-sample-stride", type=int, default=10)
    parser.add_argument("--quality-self-intersection-max-points", type=int, default=220)
    parser.add_argument("--world-launch", default=None)
    parser.add_argument("--planning-launch", default=None)
    parser.add_argument("--keep-launch-on-failure", action="store_true")
    parser.add_argument("--use-minco-final-trajectory-visualizer", type=parse_bool, default=True,
                        help="Publish a clean red Marker line for the final MINCO position trajectory during eval/replay.")
    parser.add_argument("--minco-final-line-width", type=float, default=0.085,
                        help="Line width for the clean final MINCO trajectory Marker.")
    parser.add_argument("--minco-reference-csv", default="",
                        help="Optional CSV trajectory drawn as the saved/reference trajectory in the MINCO final visualizer.")
    parser.add_argument("--minco-reference-line-width", type=float, default=0.140,
                        help="Line width for --minco-reference-csv in the MINCO final visualizer.")
    parser.add_argument("--clear-optimizer-markers", type=parse_bool, default=False,
                        help="Clear stale /scene/irl_fd_optimizer_markers before replay visualization.")
    parser.add_argument("--stream-planning-debug", type=parse_bool, default=True,
                        help="Forward sampled [MINCO-IRL] planning.log lines to stdout.")
    parser.add_argument("--stream-planning-debug-stride", type=int, default=20,
                        help="Forward every Nth [MINCO-IRL] line to stdout; planning.log keeps all lines.")
    return parser


def prepare_s_guide(args, run_dir, scene, cases):
    if args.s_guide_mode == "adaptive":
        if len(cases) != 1:
            raise ValueError("--s-guide-mode adaptive supports one d1,d2 case per run.")
        d1, d2 = cases[0]
        path, metadata = build_adaptive_s_guide(
            scene,
            d1,
            d2,
            sphere_radius=args.sphere_radius,
            clearance_buffer=args.s_guide_clearance_buffer,
            min_clearance=args.s_guide_min_clearance,
            max_clearance=args.s_guide_max_clearance,
        )
        metadata.update(guide_clearance_stats(path, scene, args.sphere_radius))
        csv_path, json_path = write_guide_artifacts(run_dir, path, metadata)
        metadata["s_guide_csv"] = os.path.abspath(csv_path)
        metadata["s_guide_json"] = os.path.abspath(json_path)
        return metadata["path_spec"], metadata

    return args.s_guide_path or guide_path_to_spec(scene["reference_path"]), {
        "path_spec": args.s_guide_path or guide_path_to_spec(scene["reference_path"]),
        "mode": "fixed",
    }


def main():
    args = build_arg_parser().parse_args()
    args.scene_config = os.path.abspath(args.scene_config)
    scene = load_scene(args.scene_config)
    cases = parse_cases(args.cases)

    run_name = args.run_name or "single_shot_{}".format(now_tag())
    run_dir = os.path.abspath(os.path.join(args.results_dir, run_name))
    os.makedirs(run_dir, exist_ok=True)
    s_guide_path, s_guide_metadata = prepare_s_guide(args, run_dir, scene, cases)

    roscore_proc = None
    if not master_is_online():
        roscore_proc = ManagedProcess("roscore", ["roscore"], os.path.join(run_dir, "roscore.log"))
        if not wait_for_master(args.master_timeout):
            raise RuntimeError("ROS master did not start within {:.1f}s".format(args.master_timeout))

    metadata = {
        "created_at": _dt.datetime.now().isoformat(),
        "mode": "single_shot_planned",
        "scene_config": args.scene_config,
        "cases": [{"d1": d1, "d2": d2} for d1, d2 in cases],
        "sphere_radius": args.sphere_radius,
        "obstacles_inflation": args.obstacles_inflation,
        "safety_zone_buffer": args.safety_zone_buffer,
        "rho_astar_waypoint": args.rho_astar_waypoint,
        "rho_astar_waypoint_stage2": args.rho_astar_waypoint_stage2,
        "minco_warm_start_cache": args.minco_warm_start_cache,
        "minco_warm_start_cache_force_reuse": args.minco_warm_start_cache_force_reuse,
        "irl_collision_weight_scale": args.irl_collision_weight_scale,
        "irl_obs_optimization_margin": args.irl_obs_optimization_margin,
        "hide_native_planning_traj": args.hide_native_planning_traj,
        "odom_source": args.odom_source,
        "odom_topic": args.odom_topic,
        "joint_state_topic": args.joint_state_topic,
        "position_cmd_topic": args.position_cmd_topic,
        "skip_world_launch": args.skip_world_launch,
        "require_start_near": args.require_start_near,
        "start_position": scene["start_position"],
        "goal_position": scene["goal_position"],
        "reference_path": scene["reference_path"],
        "s_guide_enable": args.s_guide_enable,
        "s_guide_bypass_astar": args.s_guide_bypass_astar,
        "s_guide_endpoint_tolerance": args.s_guide_endpoint_tolerance,
        "s_guide_mode": args.s_guide_mode,
        "s_guide_path": s_guide_path,
        "s_guide_metadata": s_guide_metadata,
        "trajectory_selection": args.trajectory_selection,
        "trajectory_collection_sec": args.trajectory_collection_sec,
        "use_minco_final_trajectory_visualizer": args.use_minco_final_trajectory_visualizer,
        "minco_final_line_width": args.minco_final_line_width,
        "minco_reference_csv": os.path.abspath(args.minco_reference_csv) if args.minco_reference_csv else "",
        "minco_reference_line_width": args.minco_reference_line_width,
        "quality_gates": {
            "min_clearance_1": args.quality_min_clearance_1,
            "min_clearance_2": args.quality_min_clearance_2,
            "max_path_length": args.quality_max_path_length,
            "max_length_ratio": args.quality_max_length_ratio,
            "max_local_turn_deg": args.quality_max_local_turn_deg,
            "max_arc_turn_deg": args.quality_max_arc_turn_deg,
            "arc_turn_spacing": args.quality_arc_turn_spacing,
            "arc_turn_half_window_m": args.quality_arc_turn_half_window_m,
            "max_self_intersections": args.quality_max_self_intersections,
            "sample_stride": args.quality_sample_stride,
            "self_intersection_max_points": args.quality_self_intersection_max_points,
        },
        "note": "This tool does not execute trajectories or publish waypoint segments; it can ask the planner to feed the fixed S guide path directly to MINCO.",
    }
    with open(os.path.join(run_dir, "run_metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    processes = []
    client = None
    rows = []
    try:
        processes = start_processes(args, run_dir, scene, s_guide_path, cases)
        rospy.init_node("airgrasp_single_shot_eval", anonymous=True, disable_signals=True)
        clear_legacy_scene_waypoint_markers()
        if args.hide_native_planning_traj:
            clear_native_planning_visuals(scene["frame_id"])
        if args.clear_optimizer_markers:
            clear_optimizer_markers()
        client = SingleShotClient(
            scene,
            args.fixed_odom_hz,
            odom_source=args.odom_source,
            odom_topic=args.odom_topic,
            joint_state_topic=args.joint_state_topic,
        )
        client.publish_hold(True)

        if not client.wait_for_connections(args.startup_timeout):
            rospy.logwarn("[single_shot] planner topic connections timed out.")
        if args.require_start_near or args.odom_source == "fixed":
            if not client.wait_for_odom_at_start(args.start_tolerance, args.startup_timeout):
                msg = "[single_shot] {} odom did not settle near start.".format(args.odom_source)
                if args.require_start_near:
                    raise RuntimeError(msg)
                rospy.logwarn(msg)
        elif not client.wait_for_any_odom(args.startup_timeout):
            raise RuntimeError("[single_shot] {} odom was not received.".format(args.odom_source))
        map_ready = wait_for_map(args.map_timeout)
        if not map_ready:
            rospy.logwarn("[single_shot] /sdf_map/esdf did not arrive before timeout.")

        for case_idx, (d1, d2) in enumerate(cases):
            case_name = "case_{:02d}_d1_{:.3f}_d2_{:.3f}".format(case_idx, d1, d2)
            case_dir = os.path.join(run_dir, case_name)
            os.makedirs(case_dir, exist_ok=True)
            rospy.loginfo("[single_shot] case %d/%d: d1=%.3f d2=%.3f",
                          case_idx + 1, len(cases), d1, d2)

            start_wall = time.time()
            trajectory = None
            client.publish_hold(True)
            rospy.sleep(args.reset_settle_sec)
            client.publish_safety(d1, d2, args.sphere_radius)
            client.begin_case()
            try:
                client.publish_goal(case_idx + 1)
                rospy.sleep(0.1)
                client.publish_hold(False)
                first_trajectory = client.wait_for_first_trajectory(args.trajectory_timeout)
                if first_trajectory is not None and args.trajectory_selection == "best_quality":
                    rospy.sleep(max(0.0, args.trajectory_collection_sec))
                client.end_case()
                client.publish_hold(True)
                rospy.sleep(args.case_settle_sec)
            finally:
                duration_sec = time.time() - start_wall
                client.end_case()
                snapshot = client.snapshot()
                quality_rows = evaluate_trajectory_qualities(
                    snapshot["trajectories"], scene, args.sphere_radius, d1, d2, args
                )
                trajectory, selection_meta = select_trajectory(
                    snapshot["trajectories"], quality_rows, args.trajectory_selection
                )
                case_success = trajectory is not None and (
                    not quality_required(args.trajectory_selection) or
                    selection_meta.get("quality_pass", False)
                )
                selected_traj_published = False
                if case_success:
                    selected_traj_published = client.publish_selected_trajectory(trajectory)
                case_meta = {
                    "case_id": case_idx,
                    "d1": d1,
                    "d2": d2,
                    "sphere_radius": args.sphere_radius,
                    "success": case_success,
                    "selected_trajectory_published": selected_traj_published,
                    "duration_sec": duration_sec,
                    "trajectory_selection": selection_meta,
                }
                write_case_outputs(case_dir, case_meta, trajectory, snapshot, quality_rows, selection_meta)
                rows.append(summarize_case(
                    case_idx, d1, d2, args.sphere_radius, scene,
                    trajectory, snapshot, duration_sec, case_dir, map_ready,
                    selection_meta, args
                ))
                write_summary(run_dir, rows)

        csv_path, json_path = write_summary(run_dir, rows)
        rospy.loginfo("[single_shot] done. summary_csv=%s summary_json=%s", csv_path, json_path)
        print("Results written to: {}".format(run_dir))
        print("Summary CSV: {}".format(csv_path))
        print("Summary JSON: {}".format(json_path))
        return 0
    finally:
        if client is not None:
            client.publish_hold(True)
            client.shutdown()
        if processes:
            stop_processes(processes)
            time.sleep(max(0.0, args.cleanup_sec))
        if roscore_proc is not None:
            roscore_proc.terminate()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
