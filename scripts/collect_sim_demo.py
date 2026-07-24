#!/usr/bin/python3

import argparse
import csv
import datetime as _dt
import json
import os
import shutil
import subprocess
import sys


DEFAULT_D1 = 0.50
DEFAULT_D2 = 0.20
DEFAULT_SPHERE_RADIUS = 0.25
DEFAULT_S_GUIDE_CLEARANCE_BUFFER = 0.10
DEFAULT_S_GUIDE_MIN_CLEARANCE = 0.12
DEFAULT_S_GUIDE_MAX_CLEARANCE = 0.80


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

    candidates.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
    for candidate in candidates:
        candidate = os.path.abspath(candidate)
        if os.path.exists(os.path.join(candidate, "config", "two_pillar_scene.yaml")):
            return candidate

    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def format_run_name(d1, d2):
    return "sim_demo_d1_{:.3f}_d2_{:.3f}_{}".format(d1, d2, now_tag())


def load_summary(summary_csv):
    with open(summary_csv, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise RuntimeError("No rows found in {}".format(summary_csv))
    if len(rows) != 1:
        raise RuntimeError("Expected one collection case, got {} rows.".format(len(rows)))
    return rows[0]


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


def coerce_summary_row(row):
    return {key: coerce_value(value) for key, value in row.items()}


def copy_demo_outputs(run_dir, summary_row, args):
    case_dir = os.path.abspath(summary_row["case_dir"])
    candidate_csv = summary_row.get("candidate_trajectory_csv") or os.path.join(case_dir, "candidate_planned_path.csv")
    executed_csv = os.path.join(case_dir, "executed_path.csv")
    source_csv = summary_row.get("source_trajectory_csv") or os.path.join(case_dir, "planned_trajectories.csv")
    case_json = os.path.join(case_dir, "case.json")

    source_demo_csv = os.path.abspath(candidate_csv)
    if not os.path.exists(source_demo_csv):
        raise RuntimeError("Single-shot candidate trajectory was not written: {}".format(source_demo_csv))

    demo_csv = os.path.join(run_dir, args.demo_csv_name)
    demo_metadata_json = os.path.join(run_dir, args.demo_metadata_name)

    shutil.copyfile(source_demo_csv, demo_csv)

    metadata = {
        "created_at": _dt.datetime.now().isoformat(),
        "demo_type": "simulated_two_pillar_s_trajectory",
        "collection_mode": "single_shot",
        "d1": args.d1,
        "d2": args.d2,
        "sphere_radius": args.sphere_radius,
        "rho_astar_waypoint": args.rho_astar_waypoint,
        "rho_astar_waypoint_stage2": args.rho_astar_waypoint_stage2,
        "minco_warm_start_cache": args.minco_warm_start_cache,
        "minco_warm_start_cache_force_reuse": args.minco_warm_start_cache_force_reuse,
        "irl_collision_weight_scale": args.irl_collision_weight_scale,
        "irl_obs_optimization_margin": args.irl_obs_optimization_margin,
        "safety_zone_buffer": args.safety_zone_buffer,
        "s_guide_mode": args.s_guide_mode,
        "s_guide_clearance_buffer": args.s_guide_clearance_buffer,
        "s_guide_min_clearance": args.s_guide_min_clearance,
        "s_guide_max_clearance": args.s_guide_max_clearance,
        "trajectory_selection": args.trajectory_selection,
        "trajectory_collection_sec": args.trajectory_collection_sec,
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
        "scene_config": os.path.abspath(args.scene_config),
        "results_dir": os.path.abspath(args.results_dir),
        "run_dir": os.path.abspath(run_dir),
        "case_dir": case_dir,
        "demo_csv": os.path.abspath(demo_csv),
        "raw_source_demo_csv": source_demo_csv,
        "raw_executed_path_csv": os.path.abspath(executed_csv),
        "candidate_trajectory_csv": os.path.abspath(candidate_csv),
        "planned_trajectories_csv": os.path.abspath(source_csv),
        "case_json": os.path.abspath(case_json),
        "summary": coerce_summary_row(summary_row),
    }
    with open(demo_metadata_json, "w") as f:
        json.dump(metadata, f, indent=2)

    return demo_csv, demo_metadata_json


def build_arg_parser():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Collect one simulated two-pillar trajectory and save it as a demo in data/demos."
    )
    parser.add_argument("--d1", type=float, default=DEFAULT_D1)
    parser.add_argument("--d2", type=float, default=DEFAULT_D2)
    parser.add_argument("--sphere-radius", type=float, default=DEFAULT_SPHERE_RADIUS)
    parser.add_argument("--scene-config", default=os.path.join(root, "config", "two_pillar_scene.yaml"))
    parser.add_argument("--results-dir", default=os.path.join(root, "data", "demos"))
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--single-shot-script", default=os.path.join(root, "scripts", "run_single_shot_planning_eval.py"))
    parser.add_argument("--obstacles-inflation", type=float, default=0.40)
    parser.add_argument("--safety-zone-buffer", type=float, default=0.0)
    parser.add_argument("--rho-astar-waypoint", type=float, default=0.0)
    parser.add_argument("--rho-astar-waypoint-stage2", type=float, default=0.0)
    parser.add_argument("--enable-tail-constraint", type=parse_bool, default=False)
    parser.add_argument("--minco-warm-start-cache", type=parse_bool, default=False)
    parser.add_argument("--minco-warm-start-cache-force-reuse", type=parse_bool, default=False)
    parser.add_argument("--irl-collision-weight-scale", type=float, default=3.0)
    parser.add_argument("--irl-obs-optimization-margin", type=float, default=0.05)
    parser.add_argument("--master-timeout", type=float, default=20.0)
    parser.add_argument("--startup-timeout", type=float, default=35.0)
    parser.add_argument("--map-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=45.0)
    parser.add_argument("--case-settle-sec", type=float, default=2.0)
    parser.add_argument("--cleanup-sec", type=float, default=2.0)
    parser.add_argument("--s-guide-enable", type=parse_bool, default=True)
    parser.add_argument("--s-guide-bypass-astar", type=parse_bool, default=True)
    parser.add_argument("--s-guide-endpoint-tolerance", type=float, default=0.45)
    parser.add_argument("--s-guide-mode", choices=["fixed", "adaptive"], default="fixed")
    parser.add_argument("--s-guide-path", default=None)
    parser.add_argument("--s-guide-clearance-buffer", type=float, default=DEFAULT_S_GUIDE_CLEARANCE_BUFFER)
    parser.add_argument("--s-guide-min-clearance", type=float, default=DEFAULT_S_GUIDE_MIN_CLEARANCE)
    parser.add_argument("--s-guide-max-clearance", type=float, default=DEFAULT_S_GUIDE_MAX_CLEARANCE)
    parser.add_argument("--trajectory-selection", choices=["first", "best_quality"], default="best_quality",
                        help="single_shot only: select the first planned trajectory or the best candidate passing quality gates.")
    parser.add_argument("--trajectory-collection-sec", type=float, default=2.0)
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
    parser.add_argument("--mock-odom-script", default=None)
    parser.add_argument("--demo-csv-name", default="demo_trajectory.csv")
    parser.add_argument("--demo-metadata-name", default="demo_metadata.json")
    parser.add_argument("--allow-failed-demo", action="store_true",
                        help="Write demo outputs even if one-shot planning did not fully succeed.")
    return parser


def append_optional_arg(cmd, name, value):
    if value is not None:
        cmd.extend([name, str(value)])


def build_single_shot_cmd(args, run_name):
    cmd = [
        "/usr/bin/python3",
        os.path.abspath(args.single_shot_script),
        "--cases", "{:.9f},{:.9f}".format(args.d1, args.d2),
        "--sphere-radius", str(args.sphere_radius),
        "--scene-config", os.path.abspath(args.scene_config),
        "--results-dir", os.path.abspath(args.results_dir),
        "--run-name", run_name,
        "--obstacles-inflation", str(args.obstacles_inflation),
        "--safety-zone-buffer", str(args.safety_zone_buffer),
        "--rho-astar-waypoint", str(args.rho_astar_waypoint),
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
        "--case-settle-sec", str(args.case_settle_sec),
        "--cleanup-sec", str(args.cleanup_sec),
        "--s-guide-enable", str(args.s_guide_enable).lower(),
        "--s-guide-bypass-astar", str(args.s_guide_bypass_astar).lower(),
        "--s-guide-endpoint-tolerance", str(args.s_guide_endpoint_tolerance),
        "--s-guide-mode", args.s_guide_mode,
        "--s-guide-clearance-buffer", str(args.s_guide_clearance_buffer),
        "--s-guide-min-clearance", str(args.s_guide_min_clearance),
        "--s-guide-max-clearance", str(args.s_guide_max_clearance),
        "--trajectory-selection", args.trajectory_selection,
        "--trajectory-collection-sec", str(args.trajectory_collection_sec),
        "--quality-min-clearance-1", str(args.quality_min_clearance_1),
        "--quality-min-clearance-2", str(args.quality_min_clearance_2),
        "--quality-max-path-length", str(args.quality_max_path_length),
        "--quality-max-length-ratio", str(args.quality_max_length_ratio),
        "--quality-max-local-turn-deg", str(args.quality_max_local_turn_deg),
        "--quality-max-arc-turn-deg", str(args.quality_max_arc_turn_deg),
        "--quality-arc-turn-spacing", str(args.quality_arc_turn_spacing),
        "--quality-arc-turn-half-window-m", str(args.quality_arc_turn_half_window_m),
        "--quality-max-self-intersections", str(args.quality_max_self_intersections),
        "--quality-sample-stride", str(args.quality_sample_stride),
        "--quality-self-intersection-max-points", str(args.quality_self_intersection_max_points),
    ]
    if args.s_guide_path is not None:
        cmd.append("--s-guide-path={}".format(args.s_guide_path))
    append_optional_arg(cmd, "--world-launch", args.world_launch)
    append_optional_arg(cmd, "--planning-launch", args.planning_launch)
    return cmd


def main():
    args = build_arg_parser().parse_args()
    run_name = args.run_name or format_run_name(args.d1, args.d2)
    run_dir = os.path.abspath(os.path.join(args.results_dir, run_name))

    cmd = build_single_shot_cmd(args, run_name)

    print("Collecting simulated one-shot demo: d1={:.3f}, d2={:.3f}".format(
        args.d1, args.d2
    ), flush=True)
    print("Run directory: {}".format(run_dir), flush=True)
    completed = subprocess.run(cmd)
    if completed.returncode != 0:
        return completed.returncode

    summary_csv = os.path.join(run_dir, "summary.csv")
    summary_row = load_summary(summary_csv)
    if summary_row.get("status") != "success" and not args.allow_failed_demo:
        print("Collection did not produce an accepted trajectory; no demo was accepted.")
        print("Summary CSV: {}".format(summary_csv))
        return 2

    demo_csv, demo_metadata_json = copy_demo_outputs(run_dir, summary_row, args)
    print("Demo trajectory CSV: {}".format(demo_csv))
    print("Demo metadata JSON: {}".format(demo_metadata_json))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
