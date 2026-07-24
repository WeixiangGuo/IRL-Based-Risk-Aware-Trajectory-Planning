import csv
import json
import math
import os

import yaml


DEFAULT_CLEARANCE_BUFFER = 0.10
DEFAULT_MIN_CLEARANCE = 0.12
DEFAULT_MAX_CLEARANCE = 0.80
PILLAR1_ARC_ANGLES_DEG = [150.0, 125.0, 100.0, 75.0, 50.0]
PILLAR2_ARC_ANGLES_DEG = [230.0, 255.0, 280.0, 305.0, 330.0]


def as_float_list(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{} must be a list of {} values.".format(name, length))
    return [float(v) for v in value]


def load_scene(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    pillar_radius = float(data.get("pillar_radius", 0.30))
    return {
        "start_position": as_float_list(data.get("start_position"), 3, "start_position"),
        "goal_position": as_float_list(data.get("goal_position"), 3, "goal_position"),
        "pillar1_center_xy": as_float_list(data.get("pillar1_center_xy"), 2, "pillar1_center_xy"),
        "pillar2_center_xy": as_float_list(data.get("pillar2_center_xy"), 2, "pillar2_center_xy"),
        "pillar_radius": pillar_radius,
        "pillar1_radius": float(data.get("pillar1_radius", pillar_radius)),
        "pillar2_radius": float(data.get("pillar2_radius", pillar_radius)),
        "pillar_z_min": float(data.get("pillar_z_min", 0.0)),
        "pillar_z_max": float(data.get("pillar_z_max", 2.0)),
    }


def guide_path_to_spec(path):
    return ";".join([
        "{:.9g},{:.9g},{:.9g}".format(float(p[0]), float(p[1]), float(p[2]))
        for p in path
    ])


def parse_guide_spec(spec):
    path = []
    for raw_item in str(spec).split(";"):
        item = raw_item.strip()
        if not item:
            continue
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 3:
            raise ValueError("Bad guide point '{}', expected x,y,z.".format(item))
        path.append([float(parts[0]), float(parts[1]), float(parts[2])])
    if len(path) < 2:
        raise ValueError("Guide path must contain at least two points.")
    return path


def clamp(value, lower, upper):
    return min(max(float(value), float(lower)), float(upper))


def target_clearance(d_value, clearance_buffer, min_clearance, max_clearance, clearance_floor=None):
    lower = float(min_clearance)
    if clearance_floor is not None:
        lower = max(lower, float(clearance_floor))
    return clamp(float(d_value) + float(clearance_buffer), lower, max_clearance)


def build_adaptive_s_guide(
    scene,
    d1,
    d2,
    sphere_radius=0.25,
    clearance_buffer=DEFAULT_CLEARANCE_BUFFER,
    min_clearance=DEFAULT_MIN_CLEARANCE,
    max_clearance=DEFAULT_MAX_CLEARANCE,
    clearance_floor_1=None,
    clearance_floor_2=None,
):
    start = list(scene["start_position"])
    goal = list(scene["goal_position"])
    left = scene["pillar1_center_xy"]
    right = scene["pillar2_center_xy"]
    obstacle_radius_1 = float(scene["pillar1_radius"]) + float(sphere_radius)
    obstacle_radius_2 = float(scene["pillar2_radius"]) + float(sphere_radius)
    c1 = target_clearance(d1, clearance_buffer, min_clearance, max_clearance, clearance_floor_1)
    c2 = target_clearance(d2, clearance_buffer, min_clearance, max_clearance, clearance_floor_2)
    required_distance_1 = obstacle_radius_1 + c1
    required_distance_2 = obstacle_radius_2 + c2
    max_angle_gap = max(
        max(abs(PILLAR1_ARC_ANGLES_DEG[i] - PILLAR1_ARC_ANGLES_DEG[i - 1])
            for i in range(1, len(PILLAR1_ARC_ANGLES_DEG))),
        max(abs(PILLAR2_ARC_ANGLES_DEG[i] - PILLAR2_ARC_ANGLES_DEG[i - 1])
            for i in range(1, len(PILLAR2_ARC_ANGLES_DEG))),
    )
    chord_safety = math.cos(math.radians(0.5 * max_angle_gap))
    r1 = required_distance_1 / chord_safety
    r2 = required_distance_2 / chord_safety
    z = 0.5 * (start[2] + goal[2])
    mid = [0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1]), z]

    def arc_points(center_xy, radius, angles_deg):
        points = []
        for angle_deg in angles_deg:
            angle = math.radians(angle_deg)
            points.append([
                center_xy[0] + radius * math.cos(angle),
                center_xy[1] + radius * math.sin(angle),
                z,
            ])
        return points

    def make_path(radius1, radius2):
        return (
            [start] +
            arc_points(left, radius1, PILLAR1_ARC_ANGLES_DEG) +
            [mid] +
            arc_points(right, radius2, PILLAR2_ARC_ANGLES_DEG) +
            [goal]
        )

    path = make_path(r1, r2)
    for _ in range(24):
        stats = guide_clearance_stats(path, scene, sphere_radius)
        changed = False
        if stats["guide_min_clearance_1"] < c1 - 1e-6:
            r1 += 1.25 * (c1 - stats["guide_min_clearance_1"])
            changed = True
        if stats["guide_min_clearance_2"] < c2 - 1e-6:
            r2 += 1.25 * (c2 - stats["guide_min_clearance_2"])
            changed = True
        if not changed:
            break
        path = make_path(r1, r2)

    return path, {
        "d1": float(d1),
        "d2": float(d2),
        "sphere_radius": float(sphere_radius),
        "clearance_buffer": float(clearance_buffer),
        "min_clearance": float(min_clearance),
        "max_clearance": float(max_clearance),
        "clearance_floor_1": None if clearance_floor_1 is None else float(clearance_floor_1),
        "clearance_floor_2": None if clearance_floor_2 is None else float(clearance_floor_2),
        "target_clearance_1": c1,
        "target_clearance_2": c2,
        "required_center_distance_1": required_distance_1,
        "required_center_distance_2": required_distance_2,
        "chord_safety": chord_safety,
        "pillar1_arc_angles_deg": PILLAR1_ARC_ANGLES_DEG,
        "pillar2_arc_angles_deg": PILLAR2_ARC_ANGLES_DEG,
        "guide_point_count": len(path),
        "centerline_radius_1": r1,
        "centerline_radius_2": r2,
        "path": path,
        "path_spec": guide_path_to_spec(path),
    }


def point_segment_distance_xy(point, a, b):
    px, py = float(point[0]), float(point[1])
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    vx = bx - ax
    vy = by - ay
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * vx + (py - ay) * vy) / denom
    t = clamp(t, 0.0, 1.0)
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def guide_clearance_stats(path, scene, sphere_radius):
    obstacle_radius_1 = float(scene["pillar1_radius"]) + float(sphere_radius)
    obstacle_radius_2 = float(scene["pillar2_radius"]) + float(sphere_radius)
    c1 = scene["pillar1_center_xy"]
    c2 = scene["pillar2_center_xy"]
    min_c1 = float("inf")
    min_c2 = float("inf")
    for idx in range(1, len(path)):
        a = path[idx - 1]
        b = path[idx]
        min_c1 = min(min_c1, point_segment_distance_xy(c1, a, b) - obstacle_radius_1)
        min_c2 = min(min_c2, point_segment_distance_xy(c2, a, b) - obstacle_radius_2)
    return {
        "guide_min_clearance_1": min_c1,
        "guide_min_clearance_2": min_c2,
    }


def build_adaptive_s_guide_from_config(
    scene_config,
    d1,
    d2,
    sphere_radius=0.25,
    clearance_buffer=DEFAULT_CLEARANCE_BUFFER,
    min_clearance=DEFAULT_MIN_CLEARANCE,
    max_clearance=DEFAULT_MAX_CLEARANCE,
    clearance_floor_1=None,
    clearance_floor_2=None,
):
    scene = load_scene(scene_config)
    path, metadata = build_adaptive_s_guide(
        scene,
        d1,
        d2,
        sphere_radius=sphere_radius,
        clearance_buffer=clearance_buffer,
        min_clearance=min_clearance,
        max_clearance=max_clearance,
        clearance_floor_1=clearance_floor_1,
        clearance_floor_2=clearance_floor_2,
    )
    metadata.update(guide_clearance_stats(path, scene, sphere_radius))
    metadata["scene_config"] = os.path.abspath(scene_config)
    return path, metadata


def write_guide_artifacts(output_dir, path, metadata, prefix="adaptive_s_guide"):
    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, "{}.csv".format(prefix))
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["point_index", "x", "y", "z"])
        writer.writeheader()
        for idx, point in enumerate(path):
            writer.writerow({
                "point_index": idx,
                "x": "{:.9f}".format(point[0]),
                "y": "{:.9f}".format(point[1]),
                "z": "{:.9f}".format(point[2]),
            })

    json_path = os.path.join(output_dir, "{}.json".format(prefix))
    with open(json_path, "w") as f:
        json.dump(metadata, f, indent=2)
    return csv_path, json_path
