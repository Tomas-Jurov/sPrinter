#!/usr/bin/env python3
import rospy
from numpy import mean, arctan2
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D, TransformStamped
from suncalc import get_position
from sprinter_srvs import GetOrientation, GetOrientationResponse
from datetime import datetime
from collections import deque
from tf2_ros import StaticTransformBroadcaster
import tf

class GPSProcessing:
    def __init__(self):
        # Flags
        self.is_done = False
        self.not_reached = True
        # Data measured
        self.q = deque()
        self.qsize = 5
        self.curr_pos = []
        # Subscribers
        self.sub_global = rospy.Subscriber("/gps/robot_global_orientation/cmd", Empty, self.callback_global)
        self.sub_pos = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps)
        self.sub_reached = rospy.Subscriber("/target/pose/reached", Bool, self.callback_reached)
        # Publishers
        self.pub_global = rospy.Publisher("/gps/robot_global_orientation/done", Bool, queue_size=1)
        self.pub = rospy.Publisher("/target/pose/cmd", Pose2D, queue_size=1)
        # Services
        # self.sun_server = rospy.Service('(/gps/get_sun_orientation)', GetOrientation, self.handle_get_sun_orientation)
        rospy.spin()

    # def handle_get_sun_orientation(self, req):
    #     date = datetime.now()
    #     pos = get_position(date, self.curr_pos[1], self.curr_pos[0])
    #     rospy.loginfo("Returning [%s, %s]"%(pos[0], pos[1]))
    #     return GetOrientationResponse()
    
    def callback_gps(self, data):
        self.q.append([data.latitude, data.longitude])
        if(len(self.q) > self.qsize):
            rospy.loginfo('Ready')
            self.q.popleft()

    def measure_position(self):
        pos = mean(self.q, axis=0)
        self.curr_pos = pos
        return Pose2D(x=pos[0], y=pos[1])
    
    def callback_reached(self, data):
        rospy.loginfo(data)
        if data.data:
            self.not_reached = False

    def to_quaternion(self, angle):
        broadcaster = StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "odom"

        static_transformStamped.transform.translation.x = float(0)
        static_transformStamped.transform.translation.y = float(0)
        static_transformStamped.transform.translation.z = float(0)

        quat = tf.transformations.quaternion_from_euler(
                   float(0),float(0),float(angle))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)

    def callback_global(self, data):
        if self.is_done:
            self.pub_global.publish(Bool(data=True))
        else:
            first = self.measure_position()
            self.pub.publish(first)
            rate = rospy.Rate(1) # 0.1hz
            while self.not_reached:
                rospy.loginfo(first)
                rate.sleep()
            second = self.measure_position()
            north_angle = arctan2(second.x - first.x, second.y - first.y)
            self.to_quaternion(north_angle)
            self.is_done = True


if __name__ == '__main__':
    try:
        rospy.init_node('gps_processing', anonymous=True)
        GPSProcessing()
    except rospy.ROSInterruptException:
        pass