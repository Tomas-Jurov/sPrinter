#!/usr/bin/env python3
from typing import Literal, Optional
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA

class GenericMarker(object):
    def __init__(self, frame : str, pose : Pose, type : Literal[2], color: ColorRGBA, id : Optional[int] = 0) -> None:
        self.marker = Marker()
        self.marker.header.frame_id = frame
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = id
        self.marker.type = type
        self.marker.action = Marker.ADD

        self.marker.pose = pose

        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color = color

        self.marker.lifetime = rospy.Duration(0)

    def get(self) -> Marker:
        return self.marker
