from typing import Literal, Optional
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class GenericMarker(Marker):
    def __init__(self, frame : str, pose : Pose, type : Literal[2], id : Optional[int] = 0) -> None:
        super().__init__()
        self.header.frame_id = frame
        self.header.stamp = rospy.get_rostime()
        self.id = id
        self.type = type
        self.action = Marker.ADD

        self.pose = pose

        self.scale.x = 1.0
        self.scale.y = 1.0
        self.scale.z = 1.0

        self.color.r = 0.0
        self.color.g = 0.0
        self.color.b = 1.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)
