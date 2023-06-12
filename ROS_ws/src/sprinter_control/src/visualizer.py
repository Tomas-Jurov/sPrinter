#!/usr/bin/env python3
from typing import Literal, Optional
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class GenericMarker(object):
    def __init__(self, frame : str, pose : Pose, type : Literal[2], id : Optional[int] = 0) -> None:
        self.marker = Marker()
        self.marker.header.frame_id = frame
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = id
        self.marker.type = type
        self.marker.action = Marker.ADD

        self.marker.pose = pose

        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

        self.marker.lifetime = rospy.Duration(0)

    def get(self) -> Marker:
        return self.marker
