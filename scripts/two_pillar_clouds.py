#!/usr/bin/env python3

import struct
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def _as_float_list(value, length, default):
    if isinstance(value, str):
        text = value.strip().strip("[]")
        if text:
            try:
                value = [float(part.strip()) for part in text.split(",")]
            except ValueError:
                value = default
        else:
            value = default
    if not isinstance(value, (list, tuple)) or len(value) != length:
        value = default
    return [float(v) for v in value]


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


def _rgb_to_float(rgb):
    r, g, b = [max(0, min(255, int(v))) for v in rgb]
    packed = (r << 16) | (g << 8) | b
    return struct.unpack("f", struct.pack("I", packed))[0]


def _solid_cylinder(center_xy, radius, z_min, z_max, resolution):
    radius = max(float(radius), 1e-3)
    resolution = max(float(resolution), 1e-3)
    z_min = float(z_min)
    z_max = max(float(z_max), z_min + resolution)
    cx, cy = center_xy

    xs = np.arange(-radius, radius + 0.5 * resolution, resolution, dtype=np.float32)
    ys = np.arange(-radius, radius + 0.5 * resolution, resolution, dtype=np.float32)
    zs = np.arange(z_min, z_max + 0.5 * resolution, resolution, dtype=np.float32)

    local_xy = []
    r2 = radius * radius
    for x in xs:
        for y in ys:
            if float(x * x + y * y) <= r2:
                local_xy.append((float(x), float(y)))

    points = []
    for z in zs:
        for x, y in local_xy:
            points.append((cx + x, cy + y, float(z)))
    return points


def _cloud_msg(points, rgb_float, frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(header, fields, [(x, y, z, rgb_float) for x, y, z in points])


def _cylinder_marker(marker_id, center_xy, radius, z_min, z_max, rgb, frame_id,
                     ns="two_pillar_scene", alpha=0.55):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = float(center_xy[0])
    marker.pose.position.y = float(center_xy[1])
    marker.pose.position.z = 0.5 * (float(z_min) + float(z_max))
    marker.pose.orientation.w = 1.0
    marker.scale.x = 2.0 * float(radius)
    marker.scale.y = 2.0 * float(radius)
    marker.scale.z = float(z_max) - float(z_min)
    marker.color.r = float(rgb[0]) / 255.0
    marker.color.g = float(rgb[1]) / 255.0
    marker.color.b = float(rgb[2]) / 255.0
    marker.color.a = float(alpha)
    return marker


def _selected_cloud(include_first, include_second, pillar1, pillar2):
    points = []
    if include_first:
        points.extend(pillar1)
    if include_second:
        points.extend(pillar2)
    return points


class SafetyState:
    def __init__(self, d1, d2, sphere_radius):
        self.lock = threading.Lock()
        self.d1 = float(d1)
        self.d2 = float(d2)
        self.sphere_radius = float(sphere_radius)
        self.version = 0

    def update(self, msg):
        with self.lock:
            self.d1 = max(0.0, float(msg.x))
            self.d2 = max(0.0, float(msg.y))
            if float(msg.z) > 0.0:
                self.sphere_radius = max(0.0, float(msg.z))
            self.version += 1

    def snapshot(self):
        with self.lock:
            return self.d1, self.d2, self.sphere_radius, self.version


def main():
    rospy.init_node("two_pillar_clouds")

    frame_id = rospy.get_param("~frame_id", rospy.get_param("frame_id", "world"))
    env_topic = rospy.get_param("~env_topic", "/pcl_from_pcd/env_pcl")
    tgt_topic = rospy.get_param("~tgt_topic", "/pcl_from_pcd/tgt_pcl")
    marker_topic = rospy.get_param("~marker_topic", "/scene/two_pillar_markers")
    hz = float(rospy.get_param("~hz", rospy.get_param("republish_hz", 2.0)))
    latch = _as_bool(rospy.get_param("~latch", True), True)
    publish_point_clouds = _as_bool(rospy.get_param("~publish_point_clouds", True), True)
    safety_zone_visualization = _as_bool(rospy.get_param("~safety_zone_visualization", True), True)
    safety_topic = rospy.get_param("~safety_topic", "/drone0/planning/irl_safety_distances")
    include_sphere_radius = _as_bool(rospy.get_param("~safety_zone_include_sphere_radius", True), True)
    safety_zone_buffer = float(rospy.get_param("~safety_zone_buffer", 0.0))
    initial_d1 = float(rospy.get_param("~initial_d1", 0.0))
    initial_d2 = float(rospy.get_param("~initial_d2", 0.0))
    initial_sphere_radius = float(rospy.get_param("~sphere_radius", 0.25))

    pillar1_center = _as_float_list(rospy.get_param("~pillar1_center_xy", rospy.get_param("pillar1_center_xy", [-1.4, 0.0])), 2, [-1.4, 0.0])
    pillar2_center = _as_float_list(rospy.get_param("~pillar2_center_xy", rospy.get_param("pillar2_center_xy", [1.4, 0.0])), 2, [1.4, 0.0])
    radius = float(rospy.get_param("~pillar_radius", rospy.get_param("pillar_radius", 0.30)))
    radius1 = float(rospy.get_param("~pillar1_radius", rospy.get_param("pillar1_radius", radius)))
    radius2 = float(rospy.get_param("~pillar2_radius", rospy.get_param("pillar2_radius", radius)))
    z_min = float(rospy.get_param("~pillar_z_min", rospy.get_param("pillar_z_min", 0.0)))
    z_max = float(rospy.get_param("~pillar_z_max", rospy.get_param("pillar_z_max", 2.0)))
    resolution = float(rospy.get_param("~pillar_resolution", rospy.get_param("pillar_resolution", 0.025)))
    pillar1_rgb = _as_float_list(rospy.get_param("~pillar1_rgb", rospy.get_param("pillar1_rgb", [40, 180, 130])), 3, [40, 180, 130])
    pillar2_rgb = _as_float_list(rospy.get_param("~pillar2_rgb", rospy.get_param("pillar2_rgb", [245, 135, 65])), 3, [245, 135, 65])

    env_include_pillar1 = _as_bool(rospy.get_param("~env_include_pillar1", True), True)
    env_include_pillar2 = _as_bool(rospy.get_param("~env_include_pillar2", False), False)
    tgt_include_pillar1 = _as_bool(rospy.get_param("~tgt_include_pillar1", False), False)
    tgt_include_pillar2 = _as_bool(rospy.get_param("~tgt_include_pillar2", True), True)

    safety_state = SafetyState(initial_d1, initial_d2, initial_sphere_radius)
    rospy.Subscriber(safety_topic, Vector3, safety_state.update, queue_size=10)

    env_pub = rospy.Publisher(env_topic, PointCloud2, queue_size=1, latch=latch)
    tgt_pub = rospy.Publisher(tgt_topic, PointCloud2, queue_size=1, latch=latch)
    marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(max(hz, 0.2))
    last_key = None
    env_points = []
    tgt_points = []
    marker_r1 = radius1
    marker_r2 = radius2

    rospy.loginfo(
        "[two_pillar_clouds] marker_only=%s, front_end_safety_obstacles=false, "
        "safety_zone_visualization=%s, pillar_radius=(%.3f, %.3f), safety_topic=%s",
        not publish_point_clouds,
        safety_zone_visualization,
        radius1,
        radius2,
        safety_topic,
    )

    while not rospy.is_shutdown():
        d1, d2, sphere_radius, _ = safety_state.snapshot()
        sphere_extra = sphere_radius if include_sphere_radius else 0.0
        safety_radius_1 = radius1 + sphere_extra + d1 + safety_zone_buffer
        safety_radius_2 = radius2 + sphere_extra + d2 + safety_zone_buffer

        key = (
            publish_point_clouds,
            round(radius1, 6),
            round(radius2, 6),
            round(safety_radius_1, 6),
            round(safety_radius_2, 6),
            round(resolution, 6),
        )
        if key != last_key:
            if publish_point_clouds:
                pillar1 = _solid_cylinder(pillar1_center, radius1, z_min, z_max, resolution)
                pillar2 = _solid_cylinder(pillar2_center, radius2, z_min, z_max, resolution)
                env_points = _selected_cloud(env_include_pillar1, env_include_pillar2, pillar1, pillar2)
                tgt_points = _selected_cloud(tgt_include_pillar1, tgt_include_pillar2, pillar1, pillar2)
            else:
                pillar1 = []
                pillar2 = []
                env_points = []
                tgt_points = []
            marker_r1 = safety_radius_1
            marker_r2 = safety_radius_2
            last_key = key
            rospy.loginfo(
                "[two_pillar_clouds] real pillar pcl radius=(%.3f, %.3f), analytic safety radii: r1=%.3f r2=%.3f "
                "(d1=%.3f d2=%.3f sphere=%.3f buffer=%.3f), env_pts=%d tgt_pts=%d",
                radius1,
                radius2,
                safety_radius_1,
                safety_radius_2,
                d1,
                d2,
                sphere_radius,
                safety_zone_buffer,
                len(env_points),
                len(tgt_points),
            )

        env_pub.publish(_cloud_msg(env_points, _rgb_to_float(pillar1_rgb), frame_id))
        tgt_pub.publish(_cloud_msg(tgt_points, _rgb_to_float(pillar2_rgb), frame_id))
        markers = [
            _cylinder_marker(1, pillar1_center, radius1, z_min, z_max, pillar1_rgb, frame_id),
            _cylinder_marker(2, pillar2_center, radius2, z_min, z_max, pillar2_rgb, frame_id),
        ]
        if safety_zone_visualization:
            markers.extend([
                _cylinder_marker(101, pillar1_center, marker_r1, z_min, z_max, pillar1_rgb, frame_id,
                                 ns="two_pillar_safety_distance", alpha=0.16),
                _cylinder_marker(102, pillar2_center, marker_r2, z_min, z_max, pillar2_rgb, frame_id,
                                 ns="two_pillar_safety_distance", alpha=0.16),
            ])
        marker_pub.publish(MarkerArray(markers=markers))
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
