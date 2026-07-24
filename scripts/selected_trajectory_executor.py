#!/usr/bin/env python3

import copy
import threading

import rospy
from quadrotor_msgs.msg import PositionCommand, PositionCommandTrajectory
from std_msgs.msg import Bool


def parse_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "1", "yes", "y", "on"):
            return True
        if text in ("false", "0", "no", "n", "off"):
            return False
    return bool(default)


class SelectedTrajectoryExecutor:
    def __init__(self):
        self.trajectory_topic = rospy.get_param("~trajectory_topic", "/irl/selected_position_command_trajectory")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/position_cmd")
        self.execute_enable_topic = rospy.get_param("~execute_enable_topic", "/irl/execute_enable")
        self.auto_execute = parse_bool(rospy.get_param("~auto_execute", False), False)
        self.stop_when_disabled = parse_bool(rospy.get_param("~stop_when_disabled", True), True)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.default_dt = float(rospy.get_param("~default_dt", 0.02))
        self.min_dt = float(rospy.get_param("~min_dt", 0.005))
        self.max_dt = float(rospy.get_param("~max_dt", 0.10))
        self.start_delay_sec = float(rospy.get_param("~start_delay_sec", 0.0))
        self.hold_final_sec = float(rospy.get_param("~hold_final_sec", 2.0))
        self.hold_hz = float(rospy.get_param("~hold_hz", 30.0))
        self.reexecute_same_id = parse_bool(rospy.get_param("~reexecute_same_id", False), False)

        self.lock = threading.Lock()
        self.latest_traj = None
        self.execute_enabled = self.auto_execute
        self.executing = False
        self.executed_ids = set()

        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=20)
        self.done_pub = rospy.Publisher("/irl/execution_done", Bool, queue_size=1, latch=True)
        self.traj_sub = rospy.Subscriber(
            self.trajectory_topic, PositionCommandTrajectory, self._trajectory_cb, queue_size=1
        )
        self.enable_sub = rospy.Subscriber(
            self.execute_enable_topic, Bool, self._enable_cb, queue_size=10
        )

        rospy.loginfo(
            "[selected_trajectory_executor] trajectory=%s cmd=%s enable=%s auto_execute=%s",
            self.trajectory_topic,
            self.cmd_topic,
            self.execute_enable_topic,
            self.auto_execute,
        )

    def _trajectory_cb(self, msg):
        with self.lock:
            self.latest_traj = copy.deepcopy(msg)
            point_count = len(msg.points)
            rospy.loginfo(
                "[selected_trajectory_executor] received trajectory id=%u points=%d duration=%.3f sample_dt=%.3f",
                msg.trajectory_id,
                point_count,
                msg.total_duration,
                msg.sample_dt,
            )
        self._maybe_start_execution()

    def _enable_cb(self, msg):
        with self.lock:
            self.execute_enabled = bool(msg.data)
            enabled = self.execute_enabled
        rospy.loginfo("[selected_trajectory_executor] execute_enable=%s", enabled)
        self._maybe_start_execution()

    def _maybe_start_execution(self):
        with self.lock:
            if self.executing:
                return
            if self.latest_traj is None:
                return
            if not self.execute_enabled:
                return
            trajectory_id = int(self.latest_traj.trajectory_id)
            if trajectory_id in self.executed_ids and not self.reexecute_same_id:
                rospy.logwarn(
                    "[selected_trajectory_executor] trajectory id=%u already executed; set ~reexecute_same_id=true to repeat.",
                    trajectory_id,
                )
                return
            traj = copy.deepcopy(self.latest_traj)
            self.executing = True

        worker = threading.Thread(target=self._execute, args=(traj,))
        worker.daemon = True
        worker.start()

    def _point_dt(self, traj, index):
        if index + 1 < len(traj.points):
            t0 = traj.points[index].header.stamp
            t1 = traj.points[index + 1].header.stamp
            if not t0.is_zero() and not t1.is_zero():
                dt = (t1 - t0).to_sec()
                if dt > 0.0:
                    return max(self.min_dt, min(self.max_dt, dt))
        if traj.sample_dt > 0.0:
            return max(self.min_dt, min(self.max_dt, float(traj.sample_dt)))
        return max(self.min_dt, min(self.max_dt, self.default_dt))

    def _prepare_cmd(self, source_cmd, trajectory_id):
        cmd = copy.deepcopy(source_cmd)
        cmd.header.stamp = rospy.Time.now()
        if not cmd.header.frame_id:
            cmd.header.frame_id = self.frame_id
        cmd.trajectory_id = trajectory_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return cmd

    def _execute(self, traj):
        trajectory_id = int(traj.trajectory_id)
        try:
            if not traj.points:
                rospy.logerr("[selected_trajectory_executor] empty trajectory id=%u; nothing to execute.", trajectory_id)
                return
            if self.start_delay_sec > 0.0:
                rospy.sleep(self.start_delay_sec)

            rospy.logwarn(
                "[selected_trajectory_executor] executing trajectory id=%u points=%d -> %s",
                trajectory_id,
                len(traj.points),
                self.cmd_topic,
            )
            self.done_pub.publish(Bool(data=False))

            for index, point in enumerate(traj.points):
                with self.lock:
                    enabled = self.execute_enabled
                if self.stop_when_disabled and not enabled:
                    rospy.logwarn(
                        "[selected_trajectory_executor] execution stopped by execute_enable=false at point %d/%d.",
                        index,
                        len(traj.points),
                    )
                    return
                self.cmd_pub.publish(self._prepare_cmd(point, trajectory_id))
                rospy.sleep(self._point_dt(traj, index))

            if self.hold_final_sec > 0.0:
                final_cmd = traj.points[-1]
                rate = rospy.Rate(max(self.hold_hz, 1.0))
                hold_end = rospy.Time.now() + rospy.Duration(self.hold_final_sec)
                while rospy.Time.now() < hold_end and not rospy.is_shutdown():
                    with self.lock:
                        enabled = self.execute_enabled
                    if self.stop_when_disabled and not enabled:
                        return
                    self.cmd_pub.publish(self._prepare_cmd(final_cmd, trajectory_id))
                    rate.sleep()

            rospy.logwarn("[selected_trajectory_executor] trajectory id=%u execution finished.", trajectory_id)
            self.done_pub.publish(Bool(data=True))
            with self.lock:
                self.executed_ids.add(trajectory_id)
        finally:
            with self.lock:
                self.executing = False


def main():
    rospy.init_node("selected_trajectory_executor")
    SelectedTrajectoryExecutor()
    rospy.spin()


if __name__ == "__main__":
    main()
