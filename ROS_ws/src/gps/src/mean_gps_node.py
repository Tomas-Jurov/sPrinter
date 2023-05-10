#!/usr/bin/env python
import rospy
from numpy import mean, arctan2
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D
#from suncalc import get_position
from datetime import datetime

class MeanGPS:

    def __init__(self):
        self.vector = []
        self.pos_m = []
        self.submean = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps)
        self.pubmean = rospy.Publisher("/gps/mean", NavSatFix, queue_size=1)
        self.sub = rospy.Subscriber("/gps/mean", NavSatFix, self.callback_pose)
        self.pub = rospy.Publisher("/target/pose/cmd", Pose2D, queue_size=1)

    def callback_gps(self, data):
        if(len(self.vector) < 100):
            new_pos = [data.status.latitude, data.status.longitude]
            self.vector.append(new_pos)
        else:
            pos = mean(self.vector, axis=0)
            self.pubmean.publish(NavSatFix(latitude=pos[0], longitude=pos[1]))
            self.vector.clear()

    def callback_pose(self, data):
        if(self.pos_m):
            north_angle = arctan2(data.status.longitude-self.pos_m[1], data.status.latitude-self.pos_m[0])
            #sun_pos = get_position(datetime.now(), data.status.longitude, data.status.latitude)
            self.pub.publish(Pose2D(data.status.longitude,data.status.latitude, north_angle))
        self.pos_m = [data.status.latitude, data.status.longitude]


if __name__ == '__main__':
    try:
        rospy.init_node('mean_gps', anonymous=True)
        MeanGPS()
    except rospy.ROSInterruptException:
        pass