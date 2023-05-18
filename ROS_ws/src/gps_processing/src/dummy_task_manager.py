#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Bool

class DummyTaskManager:
    def __init__(self):
        self.pub = rospy.Publisher("/gps/robot_global_orientation/cmd", Empty, queue_size=1)
        self.pub2 = rospy.Publisher("/target/pose/reached", Bool, queue_size=1)
        self.dummy_task_manager()

    def dummy_task_manager(self):
        rate = rospy.Rate(0.1) # 0.1hz
        i = 0
        while not rospy.is_shutdown():
            self.pub.publish(Empty())
            rospy.loginfo('Publishing...')
            if i == 3:
                i=0
                self.pub2.publish(Bool(data=True))
                rospy.loginfo('pose reached!...')
            i += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('DummyTaskManager', anonymous=True)
        DummyTaskManager()
    except rospy.ROSInterruptException:
        pass