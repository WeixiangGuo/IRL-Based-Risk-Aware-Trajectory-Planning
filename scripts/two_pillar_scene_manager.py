#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import UAMFullState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, Int32
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


def _quat_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def _pose(position, yaw):
    pose = Pose()
    pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
    pose.orientation = _quat_from_yaw(yaw)
    return pose


def _uam_state(seq, frame_id, position, yaw, theta, dtheta):
    msg = UAMFullState()
    msg.header = Header(seq=int(seq), stamp=rospy.Time.now(), frame_id=frame_id)
    msg.pose = _pose(position, yaw)
    msg.theta = list(theta)
    msg.dtheta = list(dtheta)
    msg.ee_pose = _pose(position, yaw)
    return msg


def _pose_stamped(frame_id, position, yaw):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose = _pose(position, yaw)
    return msg


def _odom(frame_id, position, yaw):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.child_frame_id = "base_link"
    msg.pose.pose = _pose(position, yaw)
    return msg


def _joint_state(theta, dtheta):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ["arm_joint_0", "arm_joint_1", "arm_joint_2"]
    msg.position = list(theta)
    msg.velocity = list(dtheta)
    msg.effort = [0.0, 0.0, 0.0]
    return msg


def _sphere_marker(marker_id, ns, position, radius, color, frame_id):
    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
    marker.pose.orientation.w = 1.0
    marker.scale.x = radius
    marker.scale.y = radius
    marker.scale.z = radius
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker


def _delete_all_marker():
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


class TwoPillarScene:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", rospy.get_param("frame_id", "world"))
        self.cycle_idx = int(rospy.get_param("~cycle_idx", 1))
        self.scene_settle_sec = float(rospy.get_param("~scene_settle_sec", rospy.get_param("scene_settle_sec", 1.0)))
        self.republish_hz = float(rospy.get_param("~republish_hz", rospy.get_param("republish_hz", 2.0)))
        self.publish_mock_state = _as_bool(rospy.get_param("~publish_mock_state", False), False)
        self.publish_uam_init_state = _as_bool(rospy.get_param("~publish_uam_init_state", True), True)
        self.publish_planner_goal = _as_bool(rospy.get_param("~publish_planner_goal", True), True)
        self.publish_planner_hold = _as_bool(rospy.get_param("~publish_planner_hold", True), True)

        self.start_position = _as_float_list(rospy.get_param("~start_position", rospy.get_param("start_position", [-3.5, 0.0, 1.2])), 3, [-3.5, 0.0, 1.2])
        self.goal_position = _as_float_list(rospy.get_param("~goal_position", rospy.get_param("goal_position", [3.5, 0.0, 1.2])), 3, [3.5, 0.0, 1.2])
        self.start_yaw = float(rospy.get_param("~start_yaw", rospy.get_param("start_yaw", 0.0)))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", rospy.get_param("goal_yaw", 0.0)))
        self.start_theta = _as_float_list(rospy.get_param("~start_theta", rospy.get_param("start_theta", [0.6, -0.55, 0.0])), 3, [0.6, -0.55, 0.0])
        self.goal_theta = _as_float_list(rospy.get_param("~goal_theta", rospy.get_param("goal_theta", [0.6, -0.55, 0.0])), 3, [0.6, -0.55, 0.0])
        self.start_dtheta = _as_float_list(rospy.get_param("~start_dtheta", rospy.get_param("start_dtheta", [0.0, 0.0, 0.0])), 3, [0.0, 0.0, 0.0])
        self.goal_dtheta = _as_float_list(rospy.get_param("~goal_dtheta", rospy.get_param("goal_dtheta", [0.0, 0.0, 0.0])), 3, [0.0, 0.0, 0.0])

        self.goal_cmd_topic = rospy.get_param("~goal_cmd_topic", "/drone0/planning/uam_state_goal_cmd")
        self.goal_vis_topic = rospy.get_param("~goal_vis_topic", "/drone0/planning/uam_state_goal_vis")
        self.goal_vis_mirror_topic = rospy.get_param("~goal_vis_mirror_topic", "/px4ctrl/uam_state_goal_vis")
        self.odom_topic = rospy.get_param("~odom_topic", "/drone0/odom")
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_state_est_sim")

        self.hold_pub = rospy.Publisher("/scene/planner_hold", Bool, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher("/scene/scene_ready", Bool, queue_size=1, latch=True)
        self.cycle_idx_pub = rospy.Publisher("/scene/cycle_idx", Int32, queue_size=1, latch=True)
        self.goal_cycle_ready_pub = rospy.Publisher("/scene/goal_cycle_ready", Int32, queue_size=1, latch=True)
        self.uam_init_pub = rospy.Publisher("/scene/uam_init_state", UAMFullState, queue_size=1, latch=True)
        self.goal_cmd_pub = rospy.Publisher(self.goal_cmd_topic, UAMFullState, queue_size=1, latch=True)
        self.goal_vis_pub = rospy.Publisher(self.goal_vis_topic, UAMFullState, queue_size=1, latch=True)
        self.goal_vis_mirror_pub = rospy.Publisher(self.goal_vis_mirror_topic, UAMFullState, queue_size=1, latch=True)
        self.start_pose_pub = rospy.Publisher("/scene/two_pillar_start_pose", PoseStamped, queue_size=1, latch=True)
        self.goal_pose_pub = rospy.Publisher("/scene/two_pillar_goal_pose", PoseStamped, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/scene/two_pillar_start_goal_markers", MarkerArray, queue_size=1, latch=True)

        self.odom_pub = None
        self.joint_pub = None
        if self.publish_mock_state:
            self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=1)
            self.joint_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=1)

        self.start_msg = _uam_state(self.cycle_idx, self.frame_id, self.start_position, self.start_yaw, self.start_theta, self.start_dtheta)
        self.goal_msg = _uam_state(self.cycle_idx, self.frame_id, self.goal_position, self.goal_yaw, self.goal_theta, self.goal_dtheta)

    def publish_static_scene(self):
        self.start_msg.header.stamp = rospy.Time.now()
        if self.publish_planner_hold:
            self.hold_pub.publish(Bool(data=True))
        self.cycle_idx_pub.publish(Int32(data=self.cycle_idx))
        if self.publish_uam_init_state:
            self.uam_init_pub.publish(self.start_msg)
        self.start_pose_pub.publish(_pose_stamped(self.frame_id, self.start_position, self.start_yaw))
        self.goal_pose_pub.publish(_pose_stamped(self.frame_id, self.goal_position, self.goal_yaw))
        self.ready_pub.publish(Bool(data=True))
        self.publish_markers()
        self.publish_mock_state_once()
        settle_end = rospy.Time.now() + rospy.Duration(max(self.scene_settle_sec, 0.0))
        settle_rate = rospy.Rate(max(self.republish_hz, 2.0))
        while not rospy.is_shutdown() and rospy.Time.now() < settle_end:
            self.publish_mock_state_once()
            settle_rate.sleep()
        if self.publish_planner_goal:
            self.publish_goal("initial")
        if self.publish_planner_hold:
            self.hold_pub.publish(Bool(data=False))
        self.goal_cycle_ready_pub.publish(Int32(data=self.cycle_idx))
        rospy.loginfo(
            "[two_pillar_scene] start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f), mock_state=%s uam_init=%s planner_goal=%s planner_hold=%s",
            self.start_position[0], self.start_position[1], self.start_position[2],
            self.goal_position[0], self.goal_position[1], self.goal_position[2],
            self.publish_mock_state,
            self.publish_uam_init_state,
            self.publish_planner_goal,
            self.publish_planner_hold,
        )

    def publish_goal(self, reason):
        msg = _uam_state(self.cycle_idx, self.frame_id, self.goal_position, self.goal_yaw, self.goal_theta, self.goal_dtheta)
        self.goal_msg = msg
        self.goal_cmd_pub.publish(msg)
        self.goal_vis_pub.publish(msg)
        self.goal_vis_mirror_pub.publish(msg)
        rospy.loginfo(
            "[two_pillar_scene] publish %s goal: pos=(%.2f, %.2f, %.2f), yaw=%.2f",
            reason,
            self.goal_position[0], self.goal_position[1], self.goal_position[2], self.goal_yaw,
        )

    def publish_mock_state_once(self):
        if self.publish_mock_state:
            self.odom_pub.publish(_odom(self.frame_id, self.start_position, self.start_yaw))
            self.joint_pub.publish(_joint_state(self.start_theta, self.start_dtheta))

    def publish_markers(self):
        markers = [
            _delete_all_marker(),
            _sphere_marker(10, "two_pillar_scene", self.start_position, 0.18, (0.1, 0.5, 1.0, 0.9), self.frame_id),
            _sphere_marker(11, "two_pillar_scene", self.goal_position, 0.18, (0.1, 0.9, 0.3, 0.9), self.frame_id),
        ]
        self.marker_pub.publish(MarkerArray(markers=markers))

    def spin(self):
        self.publish_static_scene()
        rate = rospy.Rate(max(self.republish_hz, 0.2))
        while not rospy.is_shutdown():
            self.publish_markers()
            self.publish_mock_state_once()
            rate.sleep()


def main():
    rospy.init_node("two_pillar_scene_manager")
    TwoPillarScene().spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
