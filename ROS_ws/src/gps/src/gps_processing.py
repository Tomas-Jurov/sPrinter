#!/usr/bin/env python
import rospy
from numpy import mean, arctan2
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D, Quaternion
from suncalc import get_position
from sprinter_srvs import GetOrientation, GetOrientationResponse
from datetime import datetime
from collections import deque

class GPSProcessing:
    def __init__(self):
        # Flags
        self.is_done = False
        self.not_reached = True
        # Data measured
        self.q = deque()
        self.qsize = 100
        self.curr_pos = []
        # Subscribers
        self.sub_global = rospy.Subscriber("/gps/robot_global_orientation/cmd", Empty, self.callback_global)
        self.sub_pos = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps)
        self.sub_reached = rospy.Subscriber("/target/pose/reached", Bool, self.callback_reached)
        # Publishers
        self.pub_global = rospy.Publisher("/gps/robot_global_orientation/done", Bool, queue_size=1)
        self.pub_pos = rospy.Publisher("/tf_static", NavSatFix, queue_size=1)
        self.pub = rospy.Publisher("/target/pose/cmd", Pose2D, queue_size=1)
        # Services
        self.sun_server = rospy.Service('(/gps/get_sun_orientation)', GetOrientation, handle_get_sun_orientation)
        rospy.spin()

    def handle_get_sun_orientation(self, req):
        date = datetime.now()
        pos = get_position(date, self.curr_pos[1], self.curr_pos[0])
        print("Returning [%s, %s]"%(pos[0], pos[1]))
        return GetOrientationResponse(Quaternion(x=pos[0], y=0, z=pos[1], w=0))
    
    def callback_gps(self, data):
        self.q.append([data.status.latitude, data.status.longitude])
        if(len(self.q) > self.qsize):
            self.q.popleft()

    def measure_position(self):
        pos = mean(self.q, axis=0)
        self.curr_pos = pos
        return Pose2D(x=pos[0], y=pos[1])
    
    def callback_reached(self, data):
        if data:
            self.is_reached = False

    def callback_global(self):
        if self.is_done:
            self.pub_global.publish(Bool(data=True))
        else:
            first = self.measure_position(self)
            self.pub.publish(first)
            while self.not_reached:
                pass
            second = self.measure_position(self)
            north_angle = arctan2(second.x - first.x, second.y - first.y)
            self.pub_pos.publish(Pose2D(second.x,second.y, north_angle))
            self.is_done = True


if __name__ == '__main__':
    try:
        rospy.init_node('gps_processing', anonymous=True)
        GPSProcessing()
    except rospy.ROSInterruptException:
        pass