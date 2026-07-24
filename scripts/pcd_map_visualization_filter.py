#!/usr/bin/env python3

import math
import os

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _parse_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "y", "on"):
            return True
        if text in ("0", "false", "no", "n", "off"):
            return False
    return default


def _read_pcd_header(f):
    header = {}
    while True:
        line = f.readline()
        if not line:
            raise RuntimeError("PCD header ended before DATA line")
        text = line.decode("latin1").strip()
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        key = parts[0].upper()
        header[key] = parts[1:]
        if key == "DATA":
            return header


def _pcd_point_step(header):
    sizes = [int(x) for x in header.get("SIZE", [])]
    counts = [int(x) for x in header.get("COUNT", ["1"] * len(sizes))]
    if len(counts) != len(sizes):
        counts = [1] * len(sizes)
    return sum(size * count for size, count in zip(sizes, counts))


def _field_offsets(header):
    fields = header.get("FIELDS", [])
    sizes = [int(x) for x in header.get("SIZE", [])]
    counts = [int(x) for x in header.get("COUNT", ["1"] * len(sizes))]
    if len(counts) != len(sizes):
        counts = [1] * len(sizes)
    offsets = {}
    offset = 0
    for name, size, count in zip(fields, sizes, counts):
        offsets[name] = offset
        offset += size * count
    return offsets


def load_xyz_from_pcd(path):
    with open(path, "rb") as f:
        header = _read_pcd_header(f)
        data_type = header.get("DATA", [""])[0].lower()
        fields = header.get("FIELDS", [])
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise RuntimeError("PCD must include x/y/z fields: {}".format(path))

        if data_type == "ascii":
            rows = []
            indices = [fields.index("x"), fields.index("y"), fields.index("z")]
            for line in f:
                parts = line.decode("latin1").strip().split()
                if len(parts) < len(fields):
                    continue
                rows.append([float(parts[idx]) for idx in indices])
            return np.asarray(rows, dtype=np.float32)

        if data_type != "binary":
            raise RuntimeError("Unsupported PCD DATA mode '{}': {}".format(data_type, path))

        points = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        point_step = _pcd_point_step(header)
        offsets = _field_offsets(header)
        blob = f.read(points * point_step)
        raw = np.frombuffer(blob, dtype=np.uint8).reshape(points, point_step)
        xyz = np.empty((points, 3), dtype=np.float32)
        for col, name in enumerate(("x", "y", "z")):
            start = offsets[name]
            xyz[:, col] = raw[:, start:start + 4].copy().view("<f4").reshape(points)
        return xyz


def filter_points(points, z_min, z_max, max_points):
    mask = np.isfinite(points).all(axis=1)
    if z_min is not None and math.isfinite(z_min):
        mask &= points[:, 2] >= float(z_min)
    if z_max is not None and math.isfinite(z_max):
        mask &= points[:, 2] <= float(z_max)

    filtered = points[mask]
    if max_points > 0 and filtered.shape[0] > max_points:
        stride = int(math.ceil(float(filtered.shape[0]) / float(max_points)))
        filtered = filtered[::stride]
    return np.ascontiguousarray(filtered.astype("<f4", copy=False))


def make_cloud(points, frame_id):
    msg = PointCloud2()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


def main():
    rospy.init_node("pcd_map_visualization_filter", anonymous=False)

    pcd_path = rospy.get_param("~pcd_path", "")
    topic = rospy.get_param("~topic", "/scene/irl_221201_map_vis_no_ceiling")
    frame_id = rospy.get_param("~frame_id", "world")
    z_min = float(rospy.get_param("~z_min", -float("inf")))
    z_max = float(rospy.get_param("~z_max", 2.0))
    max_points = int(rospy.get_param("~max_points", 180000))
    hz = float(rospy.get_param("~hz", 1.0))
    latch = _parse_bool(rospy.get_param("~latch", True), True)
    loop = _parse_bool(rospy.get_param("~loop", True), True)

    if not pcd_path:
        raise RuntimeError("~pcd_path is required")
    if not os.path.exists(pcd_path):
        raise RuntimeError("PCD does not exist: {}".format(pcd_path))

    points = load_xyz_from_pcd(pcd_path)
    filtered = filter_points(points, z_min, z_max, max_points)
    rospy.loginfo(
        "[pcd_map_visualization_filter] %s raw=%d filtered=%d z_min=%.3f z_max=%.3f max_points=%d topic=%s",
        pcd_path,
        points.shape[0],
        filtered.shape[0],
        z_min,
        z_max,
        max_points,
        topic,
    )

    pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=latch)
    rate = rospy.Rate(max(hz, 0.1))

    if loop:
        while not rospy.is_shutdown():
            pub.publish(make_cloud(filtered, frame_id))
            rate.sleep()
    else:
        pub.publish(make_cloud(filtered, frame_id))
        rospy.spin()


if __name__ == "__main__":
    main()
