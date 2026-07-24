cmake_minimum_required(VERSION 3.0.2)
project(airgrasp_minco_irl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/collect_sim_demo.py
  scripts/minco_final_trajectory_visualizer.py
  scripts/pcd_map_visualization_filter.py
  scripts/run_final_d_replay.py
  scripts/run_fd_irl_optimizer.py
  scripts/run_single_shot_planning_eval.py
  scripts/two_pillar_clouds.py
  scripts/two_pillar_scene_manager.py
  scripts/visualize_demo_guide.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
