#!/usr/bin/env python3

import math
import csv
import os

import rospy
from geometry_msgs.msg import Point
from quadrotor_msgs.msg import PositionCommandTrajectory
from visualization_msgs.msg import Marker, MarkerArray


def _as_bool(value, default=False):
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


class MincoFinalTrajectoryVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", rospy.get_param("frame_id", "world"))
        self.traj_topic = rospy.get_param(
            "~traj_topic",
            "/drone0/planning/planned_position_command_trajectory",
        )
        self.marker_topic = rospy.get_param("~marker_topic", "/scene/planned_trajectory_segments")
        self.sample_stride = max(1, int(rospy.get_param("~sample_stride", 1)))
        self.line_width = float(rospy.get_param("~line_width", 0.085))
        self.alpha = float(rospy.get_param("~alpha", 1.0))
        self.min_points = max(2, int(rospy.get_param("~min_points", 20)))
        self.min_path_length = float(rospy.get_param("~min_path_length", 0.10))
        self.show_direction_ticks = _as_bool(rospy.get_param("~show_direction_ticks", False), False)
        self.direction_tick_stride = max(1, int(rospy.get_param("~direction_tick_stride", 80)))
        self.direction_tick_length = float(rospy.get_param("~direction_tick_length", 0.16))
        self.direction_tick_width = float(rospy.get_param("~direction_tick_width", 0.025))
        self.reference_csv = str(rospy.get_param("~reference_csv", "")).strip()
        self.reference_line_width = float(rospy.get_param("~reference_line_width", self.line_width * 1.45))
        self.reference_alpha = float(rospy.get_param("~reference_alpha", 0.88))
        self.reference_z_offset = float(rospy.get_param("~reference_z_offset", 0.025))

        self.last_traj_id = None
        self.last_signature = None
        self.reference_points = self.load_reference_points(self.reference_csv)

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.traj_sub = rospy.Subscriber(
            self.traj_topic,
            PositionCommandTrajectory,
            self.traj_callback,
            queue_size=10,
        )
        self.publish_reference_only()

        rospy.loginfo(
            "[minco_final_trajectory_visualizer] traj_topic=%s marker_topic=%s line_width=%.3f reference=%s",
            self.traj_topic,
            self.marker_topic,
            self.line_width,
            self.reference_csv if self.reference_points else "<none>",
        )

    def traj_callback(self, msg):
        if len(msg.points) < self.min_points:
            rospy.logdebug(
                "[minco_final_trajectory_visualizer] ignore short trajectory id=%d points=%d",
                msg.trajectory_id,
                len(msg.points),
            )
            return

        points = self.extract_points(msg)
        path_length = self.path_length(points)
        if path_length < self.min_path_length:
            rospy.logdebug(
                "[minco_final_trajectory_visualizer] ignore tiny trajectory id=%d length=%.4f",
                msg.trajectory_id,
                path_length,
            )
            return

        signature = self.trajectory_signature(msg, points)
        if self.last_traj_id == msg.trajectory_id and self.last_signature == signature:
            rospy.logdebug(
                "[minco_final_trajectory_visualizer] ignore repeated trajectory id=%d",
                msg.trajectory_id,
            )
            return

        self.last_traj_id = msg.trajectory_id
        self.last_signature = signature

        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        markers = self.base_markers(frame_id)
        markers.append(self.line_marker(frame_id, points))
        if self.show_direction_ticks:
            markers.extend(self.direction_tick_markers(frame_id, points))
        self.marker_pub.publish(MarkerArray(markers=markers))

        rospy.loginfo(
            "[minco_final_trajectory_visualizer] show MINCO final trajectory id=%d points=%d length=%.3f",
            msg.trajectory_id,
            len(points),
            path_length,
        )

    def extract_points(self, msg):
        points = [
            Point(x=pt.position.x, y=pt.position.y, z=pt.position.z)
            for pt in msg.points[::self.sample_stride]
        ]
        end = msg.points[-1].position
        if (
            abs(points[-1].x - end.x) > 1e-9 or
            abs(points[-1].y - end.y) > 1e-9 or
            abs(points[-1].z - end.z) > 1e-9
        ):
            points.append(Point(x=end.x, y=end.y, z=end.z))
        return points

    @staticmethod
    def path_length(points):
        total = 0.0
        for left, right in zip(points, points[1:]):
            total += math.sqrt(
                (right.x - left.x) ** 2 +
                (right.y - left.y) ** 2 +
                (right.z - left.z) ** 2
            )
        return total

    @staticmethod
    def trajectory_signature(msg, points):
        first = points[0]
        last = points[-1]
        return (
            len(points),
            round(first.x, 6),
            round(first.y, 6),
            round(first.z, 6),
            round(last.x, 6),
            round(last.y, 6),
            round(last.z, 6),
            round(msg.header.stamp.to_sec(), 6),
        )

    @staticmethod
    def delete_all_marker():
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def publish_clear(self):
        self.marker_pub.publish(MarkerArray(markers=[self.delete_all_marker()]))

    def publish_reference_only(self):
        self.marker_pub.publish(MarkerArray(markers=self.base_markers(self.frame_id)))

    def base_markers(self, frame_id):
        markers = [self.delete_all_marker()]
        if self.reference_points:
            markers.append(self.reference_line_marker(frame_id, self.reference_points))
        return markers

    def line_marker(self, frame_id, points):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = "minco_final_position_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = 1.0
        marker.color.g = 0.02
        marker.color.b = 0.02
        marker.color.a = self.alpha
        marker.points = points
        return marker

    def reference_line_marker(self, frame_id, points):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = "minco_saved_reference_trajectory"
        marker.id = 10
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.reference_line_width
        marker.color.r = 0.0
        marker.color.g = 0.70
        marker.color.b = 1.0
        marker.color.a = self.reference_alpha
        marker.points = [
            Point(x=point.x, y=point.y, z=point.z + self.reference_z_offset)
            for point in points
        ]
        return marker

    def load_reference_points(self, csv_path):
        if not csv_path:
            return []
        if not os.path.exists(csv_path):
            rospy.logwarn("[minco_final_trajectory_visualizer] reference CSV does not exist: %s", csv_path)
            return []

        points = []
        try:
            with open(csv_path, "r", newline="") as f:
                reader = csv.DictReader(f)
                for idx, row in enumerate(reader):
                    if idx % self.sample_stride != 0:
                        continue
                    try:
                        points.append(Point(
                            x=float(row["x"]),
                            y=float(row["y"]),
                            z=float(row["z"]),
                        ))
                    except (KeyError, TypeError, ValueError):
                        rospy.logwarn(
                            "[minco_final_trajectory_visualizer] skip malformed reference row %d in %s",
                            idx,
                            csv_path,
                        )
            if len(points) < 2:
                rospy.logwarn(
                    "[minco_final_trajectory_visualizer] reference CSV has too few points: %s",
                    csv_path,
                )
                return []
            rospy.loginfo(
                "[minco_final_trajectory_visualizer] loaded reference trajectory points=%d from %s",
                len(points),
                csv_path,
            )
            return points
        except OSError as exc:
            rospy.logwarn(
                "[minco_final_trajectory_visualizer] failed to read reference CSV %s: %s",
                csv_path,
                exc,
            )
            return []

    def direction_tick_markers(self, frame_id, points):
        markers = []
        marker_id = 1000
        for idx in range(0, len(points) - 1, self.direction_tick_stride):
            start = points[idx]
            end = points[idx + 1]
            dx = end.x - start.x
            dy = end.y - start.y
            norm = math.sqrt(dx * dx + dy * dy)
            if norm < 1e-6:
                continue
            ux = dx / norm
            uy = dy / norm
            tick = Marker()
            tick.header.stamp = rospy.Time.now()
            tick.header.frame_id = frame_id
            tick.ns = "minco_final_direction_ticks"
            tick.id = marker_id
            tick.type = Marker.LINE_STRIP
            tick.action = Marker.ADD
            tick.pose.orientation.w = 1.0
            tick.scale.x = self.direction_tick_width
            tick.color.r = 0.70
            tick.color.g = 0.0
            tick.color.b = 0.0
            tick.color.a = min(1.0, self.alpha)
            tick.points = [
                Point(x=start.x, y=start.y, z=start.z + 0.015),
                Point(
                    x=start.x + ux * self.direction_tick_length,
                    y=start.y + uy * self.direction_tick_length,
                    z=start.z + 0.015,
                ),
            ]
            markers.append(tick)
            marker_id += 1
        return markers


def main():
    rospy.init_node("minco_final_trajectory_visualizer")
    MincoFinalTrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
