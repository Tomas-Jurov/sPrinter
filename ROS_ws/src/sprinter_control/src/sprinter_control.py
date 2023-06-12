#!/usr/bin/env python3

from __future__ import annotations 
import rospy
import tf.transformations as tf
from rospy import Publisher, ServiceProxy
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticStatus
from visualizer import GenericMarker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion

def pose2d_to_quaternion(pose : Pose2D) -> Quaternion:
    quaternion = tf.quaternion_from_euler(0, 0, pose.theta)
    return Quaternion(*quaternion)

class SprinterControl(object):
    HEARTBEAT_INTERVAL = 1.0


    def __init__(self, safety_stop_pub : Publisher, heartbeat_pub : Publisher, pos_marker_pub : Publisher, 
                 printer_point_marker_pub : Publisher, initialize_client : ServiceProxy,
                 set_pose_client : ServiceProxy, set_printer_client : ServiceProxy) -> None:
        self.safety_stop_pub = safety_stop_pub
        self.heartbeat_pub = heartbeat_pub
        self.pos_marker_pub = pos_marker_pub
        self.printer_point_marker_pub = printer_point_marker_pub
        self.initialize_client = initialize_client
        self.set_pose_client = set_pose_client
        self.set_printer_client = set_printer_client
        self.heartbeat_last = rospy.Time.now()

    def task_manager_status_callback(self, msg : DiagnosticStatus) -> None:
        if msg.level == DiagnosticStatus.OK:
            rospy.loginfo("%s: %s", msg.name, msg.message)
        elif msg.level == DiagnosticStatus.WARN:
            rospy.logwarn("%s: %s", msg.name, msg.message)
        elif msg.level == DiagnosticStatus.ERROR:
            rospy.logerr("%s: %s", msg.name, msg.message)
        else:
            rospy.logdebug("%s: %s", msg.name, msg.message)

    def target_pose_cmd_callback(self, msg : Pose2D) -> None:
        pose = Pose()
        pose.position.x = msg.x
        pose.position.y = msg.y
        pose.orientation = pose2d_to_quaternion(msg)
        pos_marker = GenericMarker('odom',pose,Marker.ARROW)
        self.pos_marker_pub.publish(pos_marker)

    def target_printer_point_cmd_callback(self, msg : Point) -> None:
        pose = Pose()
        pose.position = msg
        printer_point_marker = GenericMarker('lens_focal_static_frame',pose,Marker.SPHERE)
        self.printer_point_marker_pub.publish(printer_point_marker)

    def update(self) -> None:
        if (rospy.Time.now() - self.heartbeat_last).to_sec() > self.HEARTBEAT_INTERVAL:
            self.heartbeat_pub.publish(Empty())
            self.heartbeat_last = rospy.Time.now()