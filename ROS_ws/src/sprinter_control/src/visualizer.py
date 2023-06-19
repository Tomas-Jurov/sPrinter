#!/usr/bin/env python3
from typing import Literal, Optional
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA

class GenericMarker(object):
    def __init__(self, frame : str, pose : Pose, scale : Vector3, type : Literal[2], color: ColorRGBA, id : Optional[int] = 0) -> None:
        self.marker = Marker()
        self.marker.header.frame_id = frame
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = id
        self.marker.type = type
        self.marker.action = Marker.ADD

        self.marker.pose = pose

        self.marker.scale = scale

        self.marker.color = color

        self.marker.lifetime = rospy.Duration(0)

    def get(self) -> Marker:
        return self.marker
