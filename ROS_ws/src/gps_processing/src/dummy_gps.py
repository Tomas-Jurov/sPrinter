#!/usr/bin/env python3
import rospy
from random import randint
from sensor_msgs.msg import NavSatFix

class DummyGPS:
    def __init__(self):
        self.pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=1)
        self.dummy_gps()

    def dummy_gps(self):
        rate = rospy.Rate(1) # 0.1hz
        while not rospy.is_shutdown():
            pos = NavSatFix(latitude=randint(0,200), longitude=randint(0,200))
            rospy.loginfo(pos)
            self.pub.publish(pos)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('DummyGPS', anonymous=True)
        DummyGPS()
    except rospy.ROSInterruptException:
        pass