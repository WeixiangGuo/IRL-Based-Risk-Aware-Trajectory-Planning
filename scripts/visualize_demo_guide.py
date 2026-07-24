#!/usr/bin/env python3

import csv
import os

import rospy
import yaml
from geometry_msgs.msg import Point
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


def _point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _load_scene(scene_config):
    with open(scene_config, "r") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise RuntimeError("Bad scene config: {}".format(scene_config))
    return data


def _load_csv_points(csv_path, flatten_z=False, flat_z=1.2):
    points = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            z = float(flat_z) if flatten_z else float(row["z"])
            points.append(_point(row["x"], row["y"], z))
    return points


def _load_guide_points(scene, flatten_z=False, flat_z=1.2):
    points = []
    for item in scene.get("reference_path", []):
        if not isinstance(item, (list, tuple)) or len(item) < 3:
            continue
        z = float(flat_z) if flatten_z else float(item[2])
        points.append(_point(item[0], item[1], z))
    return points


def _line_marker(frame_id, ns, marker_id, points, rgba, width):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(width)
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])
    marker.points = list(points)
    return marker


def _sphere_list_marker(frame_id, ns, marker_id, points, rgba, diameter):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(diameter)
    marker.scale.y = float(diameter)
    marker.scale.z = float(diameter)
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])
    marker.points = list(points)
    return marker


def _text_marker(frame_id, ns, marker_id, point, text, rgba, height):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z + 0.22
    marker.pose.orientation.w = 1.0
    marker.scale.z = float(height)
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3])
    marker.text = text
    return marker


class DemoGuideVisualizer:
    def __init__(self):
        self.scene_config = rospy.get_param("~scene_config", "")
        self.demo_csv = rospy.get_param("~demo_csv", "")
        self.marker_topic = rospy.get_param("~marker_topic", "/scene/irl_demo_guide_markers")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.demo_flatten_z = _as_bool(rospy.get_param("~demo_flatten_z", False), False)
        self.guide_flatten_z = _as_bool(rospy.get_param("~guide_flatten_z", False), False)
        self.flat_z = float(rospy.get_param("~flat_z", 1.2))
        self.demo_line_width = float(rospy.get_param("~demo_line_width", 0.055))
        self.guide_line_width = float(rospy.get_param("~guide_line_width", 0.035))
        self.guide_point_diameter = float(rospy.get_param("~guide_point_diameter", 0.13))
        self.start_goal_diameter = float(rospy.get_param("~start_goal_diameter", 0.20))
        self.publish_hz = float(rospy.get_param("~publish_hz", 1.0))

        if not self.scene_config or not os.path.exists(self.scene_config):
            raise RuntimeError("scene_config not found: {}".format(self.scene_config))
        if not self.demo_csv or not os.path.exists(self.demo_csv):
            raise RuntimeError("demo_csv not found: {}".format(self.demo_csv))

        scene = _load_scene(self.scene_config)
        self.frame_id = str(scene.get("frame_id", self.frame_id))
        self.guide_points = _load_guide_points(
            scene,
            flatten_z=self.guide_flatten_z,
            flat_z=self.flat_z,
        )
        self.demo_points = _load_csv_points(
            self.demo_csv,
            flatten_z=self.demo_flatten_z,
            flat_z=self.flat_z,
        )
        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)

        rospy.loginfo(
            "[visualize_demo_guide] scene=%s demo=%s guide_points=%d demo_points=%d topic=%s",
            self.scene_config,
            self.demo_csv,
            len(self.guide_points),
            len(self.demo_points),
            self.marker_topic,
        )

    def markers(self):
        markers = []
        markers.append(_line_marker(
            self.frame_id,
            "irl_demo_guide",
            0,
            self.demo_points,
            (1.0, 0.05, 0.05, 0.92),
            self.demo_line_width,
        ))
        markers.append(_line_marker(
            self.frame_id,
            "irl_demo_guide",
            1,
            self.guide_points,
            (0.05, 0.18, 1.0, 0.95),
            self.guide_line_width,
        ))
        markers.append(_sphere_list_marker(
            self.frame_id,
            "irl_demo_guide",
            2,
            self.guide_points,
            (0.05, 0.18, 1.0, 0.90),
            self.guide_point_diameter,
        ))
        if self.guide_points:
            markers.append(_sphere_list_marker(
                self.frame_id,
                "irl_demo_guide",
                3,
                [self.guide_points[0], self.guide_points[-1]],
                (0.0, 0.85, 0.20, 0.95),
                self.start_goal_diameter,
            ))
            markers.append(_text_marker(
                self.frame_id,
                "irl_demo_guide",
                4,
                self.guide_points[0],
                "start",
                (0.0, 0.55, 0.12, 0.95),
                0.22,
            ))
            markers.append(_text_marker(
                self.frame_id,
                "irl_demo_guide",
                5,
                self.guide_points[-1],
                "goal",
                (0.0, 0.55, 0.12, 0.95),
                0.22,
            ))
        return MarkerArray(markers=markers)

    def spin(self):
        rate = rospy.Rate(max(0.2, self.publish_hz))
        while not rospy.is_shutdown():
            self.pub.publish(self.markers())
            rate.sleep()


def main():
    rospy.init_node("visualize_demo_guide", anonymous=False)
    DemoGuideVisualizer().spin()


if __name__ == "__main__":
    main()
