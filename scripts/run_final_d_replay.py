#!/usr/bin/python3

import argparse
import csv
import datetime as _dt
import json
import math
import os
import shutil
import subprocess
import sys


def _prepend_source_path():
    pkg_root = package_root_from_this_file()
    src_path = os.path.join(pkg_root, "src")
    if os.path.isdir(src_path) and src_path not in sys.path:
        sys.path.insert(0, src_path)


def package_root_from_this_file():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


_prepend_source_path()

from airgrasp_minco_irl.fd_optimizer import (  # noqa: E402
    read_trajectory,
    resample_by_path_length,
    trajectory_loss,
)


def now_tag():
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "y", "on"):
        return True
    if text in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("Expected a boolean value, got '{}'.".format(value))


def read_json(path, default=None):
    if not path or not os.path.exists(path):
        return default
    with open(path, "r") as f:
        return json.load(f)


def read_csv_rows(path):
    with open(path, "r", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, fields):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


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


def compact(value, digits=4):
    number = finite_float(value)
    if number is None:
        return "-"
    return "{:.{digits}f}".format(number, digits=digits)


def resolve_optimizer_paths(args):
    final_result_path = args.final_result_json
    optimizer_run_dir = args.optimizer_run_dir
    if final_result_path:
        final_result_path = os.path.abspath(final_result_path)
        if optimizer_run_dir is None:
            optimizer_run_dir = os.path.dirname(final_result_path)
    elif optimizer_run_dir:
        optimizer_run_dir = os.path.abspath(optimizer_run_dir)
        final_result_path = os.path.join(optimizer_run_dir, "final_result.json")

    final_result = read_json(final_result_path, {}) if final_result_path else {}
    if optimizer_run_dir is None and final_result.get("run_dir"):
        optimizer_run_dir = os.path.abspath(final_result["run_dir"])

    metadata = {}
    if optimizer_run_dir:
        metadata = read_json(os.path.join(optimizer_run_dir, "optimizer_metadata.json"), {}) or {}

    return optimizer_run_dir, final_result_path, final_result, metadata


def choose_d_values(args, final_result):
    best = final_result.get("best") or {}
    if args.d1 is not None and args.d2 is not None:
        return float(args.d1), float(args.d2), "override"

    if args.d_source == "best" and best:
        return float(best["d1"]), float(best["d2"]), "best"

    if "final_d1" in final_result and "final_d2" in final_result:
        return float(final_result["final_d1"]), float(final_result["final_d2"]), "final"

    if best:
        return float(best["d1"]), float(best["d2"]), "best_fallback"

    raise ValueError("No d1/d2 found. Provide --d1 and --d2, or pass an optimizer final_result.json.")


def inherited(args, metadata, best, name, default=None):
    value = getattr(args, name, None)
    if value is not None:
        return value
    if name in metadata and metadata[name] not in (None, ""):
        return metadata[name]
    if name in best and best[name] not in (None, ""):
        return best[name]
    return default


def bool_text(value):
    return str(bool(value)).lower()


def case_spec(d1, d2, count):
    item = "{:.9g},{:.9g}".format(float(d1), float(d2))
    return ";".join([item for _ in range(int(count))])


def build_single_shot_command(config, cases, child_results_dir, child_run_name):
    cmd = [
        "/usr/bin/python3",
        config["single_shot_script"],
        "--cases",
        cases,
        "--sphere-radius",
        str(config["sphere_radius"]),
        "--scene-config",
        config["scene_config"],
        "--results-dir",
        child_results_dir,
        "--run-name",
        child_run_name,
        "--obstacles-inflation",
        str(config["obstacles_inflation"]),
        "--safety-zone-buffer",
        str(config["safety_zone_buffer"]),
        "--rho-astar-waypoint",
        str(config["rho_astar_waypoint"]),
        "--rho-astar-waypoint-stage2",
        str(config["rho_astar_waypoint_stage2"]),
        "--enable-tail-constraint",
        bool_text(config["enable_tail_constraint"]),
        "--minco-warm-start-cache",
        bool_text(config["minco_warm_start_cache"]),
        "--minco-warm-start-cache-force-reuse",
        bool_text(config["minco_warm_start_cache_force_reuse"]),
        "--irl-collision-weight-scale",
        str(config["irl_collision_weight_scale"]),
        "--irl-obs-optimization-margin",
        str(config["irl_obs_optimization_margin"]),
        "--hide-native-planning-traj",
        bool_text(config["hide_native_planning_traj"]),
        "--master-timeout",
        str(config["master_timeout"]),
        "--startup-timeout",
        str(config["startup_timeout"]),
        "--map-timeout",
        str(config["map_timeout"]),
        "--trajectory-timeout",
        str(config["trajectory_timeout"]),
        "--reset-settle-sec",
        str(config["reset_settle_sec"]),
        "--case-settle-sec",
        str(config["case_settle_sec"]),
        "--cleanup-sec",
        str(config["cleanup_sec"]),
        "--fixed-odom-hz",
        str(config["fixed_odom_hz"]),
        "--start-tolerance",
        str(config["start_tolerance"]),
        "--s-guide-enable",
        bool_text(config["s_guide_enable"]),
        "--s-guide-bypass-astar",
        bool_text(config["s_guide_bypass_astar"]),
        "--s-guide-endpoint-tolerance",
        str(config["s_guide_endpoint_tolerance"]),
        "--s-guide-mode",
        config["s_guide_mode"],
        "--trajectory-selection",
        config["trajectory_selection"],
        "--trajectory-collection-sec",
        str(config["trajectory_collection_sec"]),
        "--quality-min-clearance-1",
        str(config["quality_min_clearance_1"]),
        "--quality-min-clearance-2",
        str(config["quality_min_clearance_2"]),
        "--quality-max-path-length",
        str(config["quality_max_path_length"]),
        "--quality-max-length-ratio",
        str(config["quality_max_length_ratio"]),
        "--quality-max-local-turn-deg",
        str(config["quality_max_local_turn_deg"]),
        "--quality-max-arc-turn-deg",
        str(config["quality_max_arc_turn_deg"]),
        "--quality-arc-turn-spacing",
        str(config["quality_arc_turn_spacing"]),
        "--quality-arc-turn-half-window-m",
        str(config["quality_arc_turn_half_window_m"]),
        "--quality-max-self-intersections",
        str(config["quality_max_self_intersections"]),
        "--quality-sample-stride",
        str(config["quality_sample_stride"]),
        "--quality-self-intersection-max-points",
        str(config["quality_self_intersection_max_points"]),
        "--use-minco-final-trajectory-visualizer",
        bool_text(config["use_minco_final_trajectory_visualizer"]),
        "--minco-final-line-width",
        str(config["minco_final_line_width"]),
        "--minco-reference-line-width",
        str(config["minco_reference_line_width"]),
        "--clear-optimizer-markers",
        "true",
        "--stream-planning-debug",
        bool_text(config["stream_planning_debug"]),
        "--stream-planning-debug-stride",
        str(config["stream_planning_debug_stride"]),
    ]
    if config.get("show_minco_reference_trajectory") and config.get("reference_csv"):
        cmd.extend(["--minco-reference-csv", config["reference_csv"]])
    if config.get("s_guide_path"):
        cmd.append("--s-guide-path={}".format(config["s_guide_path"]))
    if config.get("world_launch"):
        cmd.extend(["--world-launch", config["world_launch"]])
    if config.get("planning_launch"):
        cmd.extend(["--planning-launch", config["planning_launch"]])
    if config.get("keep_launch_on_failure"):
        cmd.append("--keep-launch-on-failure")
    return cmd


def run_command(cmd, stdout_log, tee_stdout=False):
    with open(stdout_log, "w") as log_file:
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
                if tee_stdout:
                    sys.stdout.write(line)
                    sys.stdout.flush()
        finally:
            if proc.stdout is not None:
                proc.stdout.close()
        return proc.wait()


def load_reference_samples(path, sample_count):
    if not path or not os.path.exists(path):
        return None
    return resample_by_path_length(read_trajectory(path), sample_count)


def add_loss_columns(row, demo_samples, reference_samples, sample_count, yaw_weight):
    candidate_csv = row.get("candidate_trajectory_csv") or ""
    if not candidate_csv or not os.path.exists(candidate_csv):
        return row

    try:
        points = read_trajectory(candidate_csv)
    except Exception as exc:
        row["loss_error"] = str(exc)
        return row

    if demo_samples is not None:
        loss = trajectory_loss(demo_samples, points, sample_count, yaw_weight)
        row["loss_to_demo"] = loss["loss"]
        row["position_loss_to_demo"] = loss["position_loss"]
        row["yaw_loss_to_demo"] = loss["yaw_loss"]
    if reference_samples is not None:
        loss = trajectory_loss(reference_samples, points, sample_count, yaw_weight)
        row["loss_to_reference"] = loss["loss"]
        row["position_loss_to_reference"] = loss["position_loss"]
        row["yaw_loss_to_reference"] = loss["yaw_loss"]
    return row


def copy_replay_trajectory(row, trajectory_dir, group, trial_index):
    candidate_csv = row.get("candidate_trajectory_csv") or ""
    if not candidate_csv or not os.path.exists(candidate_csv):
        return ""
    os.makedirs(trajectory_dir, exist_ok=True)
    dst = os.path.join(
        trajectory_dir,
        "{}_trial_{:02d}_case_{:02d}.csv".format(
            group,
            int(trial_index),
            int(float(row.get("case_id", trial_index) or trial_index)),
        ),
    )
    shutil.copyfile(candidate_csv, dst)
    return os.path.abspath(dst)


def collect_child_rows(child_run_dir, group, trial_offset, demo_samples, reference_samples,
                       sample_count, yaw_weight, trajectory_dir):
    summary_csv = os.path.join(child_run_dir, "summary.csv")
    if not os.path.exists(summary_csv):
        raise RuntimeError("Missing child summary.csv: {}".format(summary_csv))

    rows = []
    for local_idx, row in enumerate(read_csv_rows(summary_csv)):
        trial_index = trial_offset + local_idx
        row = dict(row)
        row["group"] = group
        row["trial_index"] = trial_index
        row["child_run_dir"] = os.path.abspath(child_run_dir)
        row = add_loss_columns(row, demo_samples, reference_samples, sample_count, yaw_weight)
        row["replay_trajectory_csv"] = copy_replay_trajectory(row, trajectory_dir, group, trial_index)
        rows.append(row)
    return rows


def union_fields(rows, preferred):
    seen = set()
    fields = []
    for field in preferred:
        if field not in seen:
            fields.append(field)
            seen.add(field)
    for row in rows:
        for field in row.keys():
            if field not in seen:
                fields.append(field)
                seen.add(field)
    return fields


def print_summary(rows):
    if not rows:
        print("[final_replay] no replay rows.")
        return
    print("")
    print("[final_replay] replay summary")
    print("  {:<12} {:>5} {:<9} {:>9} {:>9} {:>8} {:>8} {:>8} {:>8} {:>10}".format(
        "group", "trial", "status", "loss_demo", "loss_ref", "clr1", "clr2", "mrg1", "mrg2", "len"
    ))
    for row in rows:
        print("  {:<12} {:>5} {:<9} {:>9} {:>9} {:>8} {:>8} {:>8} {:>8} {:>10}".format(
            str(row.get("group", "-"))[:12],
            row.get("trial_index", "-"),
            str(row.get("status", "-"))[:9],
            compact(row.get("loss_to_demo"), 5),
            compact(row.get("loss_to_reference"), 5),
            compact(row.get("candidate_min_clearance_1"), 3),
            compact(row.get("candidate_min_clearance_2"), 3),
            compact(row.get("candidate_min_margin_1"), 3),
            compact(row.get("candidate_min_margin_2"), 3),
            compact(row.get("candidate_path_length"), 3),
        ))


def build_config(args, final_result, metadata, optimizer_run_dir, d1, d2):
    root = package_root_from_this_file()
    best = final_result.get("best") or {}
    scene_config = args.scene_config or metadata.get("scene_config") or os.path.join(root, "config", "two_pillar_scene.yaml")
    demo_csv = args.demo_csv or final_result.get("demo_csv") or metadata.get("demo_csv")
    reference_csv = args.reference_csv
    if reference_csv is None and final_result.get("best"):
        reference_csv = best.get("best_trajectory_csv") or best.get("trajectory_csv")
    if reference_csv is None and optimizer_run_dir:
        candidate = os.path.join(optimizer_run_dir, "best_trajectory.csv")
        if os.path.exists(candidate):
            reference_csv = candidate

    s_guide_path = args.s_guide_path
    if s_guide_path is None:
        s_guide_path = best.get("s_guide_path") or metadata.get("s_guide_path")
    if s_guide_path == "":
        s_guide_path = None

    quality_gates = metadata.get("quality_gates") or {}
    return {
        "d1": d1,
        "d2": d2,
        "demo_csv": os.path.abspath(demo_csv) if demo_csv else "",
        "reference_csv": os.path.abspath(reference_csv) if reference_csv else "",
        "single_shot_script": os.path.abspath(args.single_shot_script or os.path.join(root, "scripts", "run_single_shot_planning_eval.py")),
        "scene_config": os.path.abspath(scene_config),
        "sphere_radius": inherited(args, metadata, best, "sphere_radius", 0.25),
        "obstacles_inflation": inherited(args, metadata, best, "obstacles_inflation", 0.40),
        "safety_zone_buffer": inherited(args, metadata, best, "safety_zone_buffer", 0.0),
        "rho_astar_waypoint": inherited(args, metadata, best, "rho_astar_waypoint", 100000.0),
        "rho_astar_waypoint_stage2": inherited(args, metadata, best, "rho_astar_waypoint_stage2", 0.0),
        "enable_tail_constraint": inherited(args, metadata, best, "enable_tail_constraint", False),
        "minco_warm_start_cache": inherited(args, metadata, best, "minco_warm_start_cache", True),
        "minco_warm_start_cache_force_reuse": inherited(args, metadata, best, "minco_warm_start_cache_force_reuse", True),
        "irl_collision_weight_scale": inherited(args, metadata, best, "irl_collision_weight_scale", 5.0),
        "irl_obs_optimization_margin": inherited(args, metadata, best, "irl_obs_optimization_margin", 0.05),
        "hide_native_planning_traj": args.hide_native_planning_traj,
        "master_timeout": args.master_timeout,
        "startup_timeout": args.startup_timeout,
        "map_timeout": args.map_timeout,
        "trajectory_timeout": args.trajectory_timeout,
        "reset_settle_sec": args.reset_settle_sec,
        "case_settle_sec": args.case_settle_sec,
        "cleanup_sec": args.cleanup_sec,
        "fixed_odom_hz": args.fixed_odom_hz,
        "start_tolerance": args.start_tolerance,
        "s_guide_enable": inherited(args, metadata, best, "s_guide_enable", True),
        "s_guide_bypass_astar": inherited(args, metadata, best, "s_guide_bypass_astar", True),
        "s_guide_endpoint_tolerance": inherited(args, metadata, best, "s_guide_endpoint_tolerance", 0.45),
        "s_guide_mode": args.s_guide_mode or metadata.get("s_guide_mode") or best.get("s_guide_mode") or "fixed",
        "s_guide_path": s_guide_path,
        "trajectory_selection": args.trajectory_selection,
        "trajectory_collection_sec": args.trajectory_collection_sec,
        "quality_min_clearance_1": args.quality_min_clearance_1 if args.quality_min_clearance_1 is not None else quality_gates.get("min_clearance_1", 0.60),
        "quality_min_clearance_2": args.quality_min_clearance_2 if args.quality_min_clearance_2 is not None else quality_gates.get("min_clearance_2", 0.28),
        "quality_max_path_length": args.quality_max_path_length if args.quality_max_path_length is not None else quality_gates.get("max_path_length", 12.5),
        "quality_max_length_ratio": args.quality_max_length_ratio if args.quality_max_length_ratio is not None else quality_gates.get("max_length_ratio", 1.65),
        "quality_max_local_turn_deg": args.quality_max_local_turn_deg if args.quality_max_local_turn_deg is not None else quality_gates.get("max_local_turn_deg", 25.0),
        "quality_max_arc_turn_deg": args.quality_max_arc_turn_deg if args.quality_max_arc_turn_deg is not None else quality_gates.get("max_arc_turn_deg", 35.0),
        "quality_arc_turn_spacing": args.quality_arc_turn_spacing,
        "quality_arc_turn_half_window_m": args.quality_arc_turn_half_window_m,
        "quality_max_self_intersections": args.quality_max_self_intersections,
        "quality_sample_stride": args.quality_sample_stride,
        "quality_self_intersection_max_points": args.quality_self_intersection_max_points,
        "world_launch": args.world_launch,
        "planning_launch": args.planning_launch,
        "keep_launch_on_failure": args.keep_launch_on_failure,
        "use_minco_final_trajectory_visualizer": args.use_minco_final_trajectory_visualizer,
        "show_minco_reference_trajectory": args.show_minco_reference_trajectory,
        "minco_final_line_width": args.minco_final_line_width,
        "minco_reference_line_width": args.minco_reference_line_width,
        "stream_planning_debug": args.stream_planning_debug,
        "stream_planning_debug_stride": args.stream_planning_debug_stride,
    }


def build_arg_parser():
    root = package_root_from_this_file()
    parser = argparse.ArgumentParser(
        description="Replay learned final d1/d2 multiple times for two-pillar MINCO-IRL validation."
    )
    parser.add_argument("--optimizer-run-dir", default=None,
                        help="Optimizer run directory containing final_result.json and optimizer_metadata.json.")
    parser.add_argument("--final-result-json", default=None,
                        help="Path to optimizer final_result.json. Overrides --optimizer-run-dir/final_result.json.")
    parser.add_argument("--d-source", choices=["final", "best"], default="final",
                        help="Use final_d1/final_d2 or best.d1/best.d2 from final_result.json.")
    parser.add_argument("--d1", type=float, default=None)
    parser.add_argument("--d2", type=float, default=None)
    parser.add_argument("--demo-csv", default=None)
    parser.add_argument("--reference-csv", default=None,
                        help="Reference trajectory for replay-to-reference loss. Default: optimizer best trajectory.")
    parser.add_argument("--results-dir", default=os.path.join(root, "results", "replays"))
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--replay-mode", choices=["same_process", "fresh", "both"], default="same_process")
    parser.add_argument("--same-process-trials", type=int, default=5)
    parser.add_argument("--fresh-trials", type=int, default=0,
                        help="Number of fresh planner restarts. If --replay-mode fresh/both and this is 0, defaults to 3.")
    parser.add_argument("--sample-count", type=int, default=200)
    parser.add_argument("--yaw-weight", type=float, default=0.0)
    parser.add_argument("--single-shot-script", default=os.path.join(root, "scripts", "run_single_shot_planning_eval.py"))
    parser.add_argument("--scene-config", default=None)
    parser.add_argument("--sphere-radius", type=float, default=None)
    parser.add_argument("--obstacles-inflation", type=float, default=None)
    parser.add_argument("--safety-zone-buffer", type=float, default=None)
    parser.add_argument("--rho-astar-waypoint", type=float, default=None)
    parser.add_argument("--rho-astar-waypoint-stage2", type=float, default=None)
    parser.add_argument("--enable-tail-constraint", type=parse_bool, default=None)
    parser.add_argument("--minco-warm-start-cache", type=parse_bool, default=None)
    parser.add_argument("--minco-warm-start-cache-force-reuse", type=parse_bool, default=None)
    parser.add_argument("--irl-collision-weight-scale", type=float, default=None)
    parser.add_argument("--irl-obs-optimization-margin", type=float, default=None)
    parser.add_argument("--hide-native-planning-traj", type=parse_bool, default=True,
                        help="Hide the planner's native blue /drone0/planning/traj Path during replay.")
    parser.add_argument("--master-timeout", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=35.0)
    parser.add_argument("--map-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=60.0)
    parser.add_argument("--reset-settle-sec", type=float, default=1.0)
    parser.add_argument("--case-settle-sec", type=float, default=2.0)
    parser.add_argument("--cleanup-sec", type=float, default=2.0)
    parser.add_argument("--fixed-odom-hz", type=float, default=50.0)
    parser.add_argument("--start-tolerance", type=float, default=0.05)
    parser.add_argument("--s-guide-enable", type=parse_bool, default=None)
    parser.add_argument("--s-guide-bypass-astar", type=parse_bool, default=None)
    parser.add_argument("--s-guide-endpoint-tolerance", type=float, default=None)
    parser.add_argument("--s-guide-mode", choices=["fixed", "adaptive"], default=None)
    parser.add_argument("--s-guide-path", default=None)
    parser.add_argument("--trajectory-selection", choices=["first", "best_quality"], default="first")
    parser.add_argument("--trajectory-collection-sec", type=float, default=2.0)
    parser.add_argument("--quality-min-clearance-1", type=float, default=None)
    parser.add_argument("--quality-min-clearance-2", type=float, default=None)
    parser.add_argument("--quality-max-path-length", type=float, default=None)
    parser.add_argument("--quality-max-length-ratio", type=float, default=None)
    parser.add_argument("--quality-max-local-turn-deg", type=float, default=None)
    parser.add_argument("--quality-max-arc-turn-deg", type=float, default=None)
    parser.add_argument("--quality-arc-turn-spacing", type=float, default=0.05)
    parser.add_argument("--quality-arc-turn-half-window-m", type=float, default=0.10)
    parser.add_argument("--quality-max-self-intersections", type=int, default=0)
    parser.add_argument("--quality-sample-stride", type=int, default=10)
    parser.add_argument("--quality-self-intersection-max-points", type=int, default=220)
    parser.add_argument("--world-launch", default=None)
    parser.add_argument("--planning-launch", default=None)
    parser.add_argument("--keep-launch-on-failure", action="store_true")
    parser.add_argument("--use-minco-final-trajectory-visualizer", type=parse_bool, default=True,
                        help="Start a clean red final MINCO trajectory visualizer inside each replay eval.")
    parser.add_argument("--show-minco-reference-trajectory", type=parse_bool, default=False,
                        help="Draw the saved/best reference trajectory in RViz together with the replayed MINCO trajectory.")
    parser.add_argument("--minco-final-line-width", type=float, default=0.085)
    parser.add_argument("--minco-reference-line-width", type=float, default=0.170)
    parser.add_argument("--stream-planning-debug", type=parse_bool, default=False)
    parser.add_argument("--stream-planning-debug-stride", type=int, default=20)
    parser.add_argument("--tee-stdout", type=parse_bool, default=False)
    parser.add_argument("--dry-run", action="store_true",
                        help="Write metadata and child commands without launching ROS/planner.")
    return parser


def main():
    args = build_arg_parser().parse_args()
    optimizer_run_dir, final_result_path, final_result, metadata = resolve_optimizer_paths(args)
    d1, d2, d_source = choose_d_values(args, final_result)
    config = build_config(args, final_result, metadata, optimizer_run_dir, d1, d2)

    if not config["demo_csv"]:
        raise ValueError("No demo CSV found. Provide --demo-csv or use an optimizer final_result.json with demo_csv.")
    if not os.path.exists(config["demo_csv"]):
        raise ValueError("Demo CSV does not exist: {}".format(config["demo_csv"]))

    same_trials = args.same_process_trials if args.replay_mode in ("same_process", "both") else 0
    fresh_trials = args.fresh_trials if args.replay_mode in ("fresh", "both") else 0
    if args.replay_mode in ("fresh", "both") and fresh_trials <= 0:
        fresh_trials = 3
    if same_trials < 0 or fresh_trials < 0:
        raise ValueError("Trial counts must be non-negative.")
    if same_trials == 0 and fresh_trials == 0:
        raise ValueError("No replay trials requested.")

    run_name = args.run_name or "final_d_replay_{}".format(now_tag())
    run_dir = os.path.abspath(os.path.join(args.results_dir, run_name))
    os.makedirs(run_dir, exist_ok=True)
    trajectory_dir = os.path.join(run_dir, "replay_trajectories")

    demo_samples = load_reference_samples(config["demo_csv"], args.sample_count)
    reference_samples = load_reference_samples(config["reference_csv"], args.sample_count)

    metadata_out = {
        "created_at": _dt.datetime.now().isoformat(),
        "mode": "final_d_replay",
        "optimizer_run_dir": os.path.abspath(optimizer_run_dir) if optimizer_run_dir else "",
        "final_result_json": os.path.abspath(final_result_path) if final_result_path else "",
        "d_source": d_source,
        "d1": d1,
        "d2": d2,
        "replay_mode": args.replay_mode,
        "same_process_trials": same_trials,
        "fresh_trials": fresh_trials,
        "sample_count": args.sample_count,
        "yaw_weight": args.yaw_weight,
        "config": config,
    }
    with open(os.path.join(run_dir, "run_metadata.json"), "w") as f:
        json.dump(metadata_out, f, indent=2)

    child_commands = []
    rows = []

    if same_trials > 0:
        child_run_name = "same_process"
        child_run_dir = os.path.join(run_dir, child_run_name)
        cmd = build_single_shot_command(
            config,
            case_spec(d1, d2, same_trials),
            run_dir,
            child_run_name,
        )
        child_commands.append({"group": "same_process", "run_dir": child_run_dir, "cmd": cmd})
        print("[final_replay] same_process trials={} d=({:.4f},{:.4f})".format(same_trials, d1, d2), flush=True)
        if not args.dry_run:
            rc = run_command(cmd, os.path.join(run_dir, "same_process_invocation.log"), tee_stdout=args.tee_stdout)
            if rc != 0:
                raise RuntimeError("same_process replay failed with returncode {}".format(rc))
            rows.extend(collect_child_rows(
                child_run_dir,
                "same_process",
                0,
                demo_samples,
                reference_samples,
                args.sample_count,
                args.yaw_weight,
                trajectory_dir,
            ))

    if fresh_trials > 0:
        for trial_idx in range(fresh_trials):
            child_run_name = "fresh_trial_{:02d}".format(trial_idx)
            child_run_dir = os.path.join(run_dir, child_run_name)
            cmd = build_single_shot_command(
                config,
                case_spec(d1, d2, 1),
                run_dir,
                child_run_name,
            )
            child_commands.append({"group": "fresh", "trial_index": trial_idx, "run_dir": child_run_dir, "cmd": cmd})
            print("[final_replay] fresh trial {}/{} d=({:.4f},{:.4f})".format(
                trial_idx + 1, fresh_trials, d1, d2
            ), flush=True)
            if args.dry_run:
                continue
            rc = run_command(cmd, os.path.join(run_dir, "{}_invocation.log".format(child_run_name)), tee_stdout=args.tee_stdout)
            if rc != 0:
                raise RuntimeError("fresh replay trial {} failed with returncode {}".format(trial_idx, rc))
            rows.extend(collect_child_rows(
                child_run_dir,
                "fresh",
                trial_idx,
                demo_samples,
                reference_samples,
                args.sample_count,
                args.yaw_weight,
                trajectory_dir,
            ))

    with open(os.path.join(run_dir, "child_commands.json"), "w") as f:
        json.dump(child_commands, f, indent=2)

    preferred_fields = [
        "group",
        "trial_index",
        "case_id",
        "d1",
        "d2",
        "status",
        "loss_to_demo",
        "position_loss_to_demo",
        "yaw_loss_to_demo",
        "loss_to_reference",
        "position_loss_to_reference",
        "yaw_loss_to_reference",
        "failure_count",
        "failure_reasons",
        "candidate_path_length",
        "candidate_min_clearance_1",
        "candidate_min_clearance_2",
        "candidate_min_margin_1",
        "candidate_min_margin_2",
        "trajectory_quality_pass",
        "trajectory_quality_score",
        "trajectory_quality_reasons",
        "candidate_max_local_turn_deg",
        "candidate_max_arc_turn_deg",
        "candidate_length_ratio",
        "candidate_self_intersection_count",
        "replay_trajectory_csv",
        "candidate_trajectory_csv",
        "source_trajectory_csv",
        "child_run_dir",
    ]
    summary_csv = os.path.join(run_dir, "replay_summary.csv")
    summary_json = os.path.join(run_dir, "replay_summary.json")
    write_csv(summary_csv, rows, union_fields(rows, preferred_fields))
    with open(summary_json, "w") as f:
        json.dump(rows, f, indent=2)

    print_summary(rows)
    print("")
    print("Final replay results written to: {}".format(run_dir), flush=True)
    print("Replay summary CSV: {}".format(summary_csv), flush=True)
    print("Replay summary JSON: {}".format(summary_json), flush=True)
    print("Child commands: {}".format(os.path.join(run_dir, "child_commands.json")), flush=True)
    if args.dry_run:
        print("[final_replay] dry run only; no planner was launched.", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
