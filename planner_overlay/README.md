# MINCO/IRL planner overlay

This directory contains the planner-side C++ implementation required by the
`airgrasp_minco_irl` ROS package. The files keep their original paths beneath
the AirGrasp planning workspace so that the integration boundary is explicit.

Key implementation points:

- `clutter_plan_node.cpp` subscribes to the runtime `d1/d2` safety-distance
  command.
- `tlplanner.cpp` builds or reuses the S-guide and invokes the 7-D MINCO
  trajectory optimizer.
- `traj_opt_s4_goal_yaw_arm.cc` implements the two-stage optimization and the
  analytic obstacle collision cost.
- `traj_opt_util.cc` manages the IRL parameters and performs the final
  trajectory safety-margin hard check.
- `map_ros_interface.hpp` separates point-cloud obstacles into semantic groups.

## Integration

Use a compatible checkout of the AirGrasp planning workspace, then merge the
contents of this directory at the workspace root while preserving paths:

```bash
rsync -av planner_overlay/ /path/to/airgrasp_planning/
```

Review local changes before building. The overlay is intentionally kept
separate from the Python/ROS package because these files belong to the existing
`planning` and `plan_env_lod` catkin packages.
