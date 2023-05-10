#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from numpy import mean
def callback(data):
    while (len(vector) < 100):
        vector.append(data)
    mean(vector.longitude)
    vector.clear()

def mean_pos_node(vector):
    rospy.init_node('mean_gps', anonymous=True)
    rospy.Subscriber("/gps/fix", String, callback)

if __name__ == '__main__':
    try:
        vector = []
        mean_pos_node(vector)
    except rospy.ROSInterruptException:
        pass