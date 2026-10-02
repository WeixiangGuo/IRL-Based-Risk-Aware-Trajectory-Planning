# IRL-Based Risk-Aware Trajectory Planning

This repository implements an inverse reinforcement learning (IRL) framework
for risk-aware trajectory planning on the AirGrasp aerial manipulator. It learns
obstacle-specific safety clearances from human flight demonstrations and uses
them in a dynamically feasible MINCO trajectory optimizer.

## Overview

The system models two semantic obstacle groups with independent safety
distances, `d1` and `d2`. An outer-loop optimizer searches for the distances
that best reproduce a demonstrated trajectory, while the inner loop generates
a collision-free trajectory subject to smoothness, time, attitude, and arm
motion constraints.

The planner supports:

- finite-difference and best-neighbor updates for `d1` and `d2`;
- fixed or adaptive S-guide initialization;
- analytic cylinder collision costs and final clearance checks;
- simulation evaluation, repeatability tests, and onboard execution;
- ROS visualization and export of optimization traces.

## Repository Structure

```text
config/           Scene and guide-path configurations
data/             Human demonstrations and point-cloud maps
launch/           ROS launch files for planning and visualization
scripts/          Data collection, optimization, replay, and execution tools
src/              IRL outer-loop implementation
results/          Reference optimization and replay results
planner_overlay/  MINCO/IRL modifications for the AirGrasp planner
```

The Python package contains the IRL experiment workflow. The planner-side C++
implementation is provided in `planner_overlay/` using its original workspace
paths. Merge that overlay into a compatible AirGrasp planning workspace before
building.

## Quick Start

Place this repository in a catkin workspace, integrate the planner overlay, and
build the workspace:

```bash
rsync -av planner_overlay/ /path/to/airgrasp_planning/
cd /path/to/airgrasp_ws
catkin_make
source devel/setup.bash
```

Launch the two-obstacle planning scene:

```bash
roslaunch airgrasp_minco_irl two_pillar_planner_only.launch
```

Run a single-shot evaluation:

```bash
rosrun airgrasp_minco_irl run_single_shot_planning_eval.py \
  --cases 0.50,0.01 \
  --s-guide-enable true \
  --s-guide-bypass-astar true
```

The main outer-loop entry point is `scripts/run_fd_irl_optimizer.py`. Example
configurations for simulation and real-world demonstrations are available under
`config/` and `data/demos/`.

## Reference Result

The archived real-world demonstration run in
`results/runs/irl_fd_manual_20260724_161550/` converged to `d1 = 0.01 m` and
`d2 = 0.22 m`. The selected trajectory passed the final feasibility and
clearance checks.

## Status

This is research code built around ROS 1 and the AirGrasp planning stack. Review
all scene frames, safety parameters, and controller topics before hardware use.
