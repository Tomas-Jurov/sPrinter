#!/usr/bin/env python3

## @package gps_processing
# Documentation for gps_processing node.

## @file gps_processing.py
# @brief Node in charge of estimating the geographical position of the robot,
# its global orientation and the position of the sun at the given time and place.
#
# @section author_doxygen_example Author(s)
# - Created by Daniel Acuña on 28/05/2023.

# Imports

## Python
from numpy import mean, arctan2, pi
from suncalc import get_position
from datetime import datetime, date
from collections import deque

## ROS
import rospy
from tf2_ros import StaticTransformBroadcaster
import tf
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import NavSatFix, TimeReference
from geometry_msgs.msg import Pose2D, TransformStamped, Quaternion
from sprinter_srvs.srv import GetOrientation

class GPSProcessing:
    """! The GPS processing class.

    Defines the GPS processing node class utilized.
    """
    def __init__(self):
        """! The GPSProcessing class initializer.
        @return  An instance of the GPSProcessing class initialized with the specified name.
        """
        ## Flag indicating if the robot has already moved to the second position.
        self.pose_reached = False

        self.global_orientation_done = False
        
        ## Queue of data taken from the GPS.
        self.q = deque()
        self.qsize = 100
        
        ## Last GPS data collection time.
        self.GPS_time = datetime.now()
        
        ## Subscribers
        self.sub_global_orientation_do = rospy.Subscriber("/gps/global_orientation/do", Empty, self.callback_global_orientation_do)
        self.sub_global_orientation_reset = rospy.Subscriber("/gps/global_orientation/reset", Empty, self.callback_global_orientation_reset)
        self.sub_pos = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps)
        self.sub_pos_t = rospy.Subscriber("/gps/time_ref", TimeReference, self.callback_gps_time)
        self.sub_reached = rospy.Subscriber("/target/pose/reached", Bool, self.callback_pose)
        
        ## Publishers
        self.pub_global = rospy.Publisher("/gps/global_orientation/done", Bool, queue_size=1)
        self.pub = rospy.Publisher("/target/pose/cmd", Pose2D, queue_size=1)
        
        ## Service server
        self.sun_server = rospy.Service('/gps/get_sun_orientation', GetOrientation, self.handle_get_sun_orientation)
        rospy.spin()

    def handle_get_sun_orientation(self, req):
        """! Manager of sun orientation service. 
        
        Based on the position and time data provided by GPS,
        calculates the azimuth angle and the height angle of the sun. 
        Then converts these angles into quaternion coordinates.
        @param req sprinter_srvs/GetOrientationRequest Empty request
        @return  Sun's angular position in "world" frame (quaternion coordinates).
        """
        if self.global_orientation_done:
            gps = self.measure_position()
            pos_sun = get_position(self.GPS_time, gps.y, gps.x)
            rospy.loginfo("Returning [%s, %s]"%(float(pos_sun['altitude']), float(pos_sun['azimuth'])+pi))
            quat = tf.transformations.quaternion_from_euler(
                    float(0),float(pos_sun['altitude']),float(pos_sun['azimuth']+pi))
            return GetOrientationResponse(True, Quaternion(*quat))
        else:
            return GetOrientationResponse(False, Quaternion(0, 0, 0, 1))
        
    def callback_gps_time(self, data):
        """! Callback of the GPS time. 
        
        Updates the last time sample.
        @param data sensor_msgs/TimeReference New sample.
        """
        self.GPS_time = datetime.utcfromtimestamp(data.time_ref.to_sec())

    def callback_gps(self, data):
        """! Callback of the GPS position. 
        
        Adds a new sample at the end of the queue. If the queue
        is full, removes the oldest sample.
        @param data sensor_msgs/NavSatFix New sample.
        """
        self.q.append([data.latitude, data.longitude])
        if(len(self.q) > self.qsize):
            self.q.popleft()

    def measure_position(self):
        """! Computes the average of the samples in the queue.
        
        @return Pose2D object with the latitude in the x attribute and 
        longitude in the y attribute.
        """
        # reseting q buffer
        while(len(self.q)): self.q.popleft()
        while(len(self.q) < self.qsize): pass
        
        pos = mean(self.q, axis=0)
        return Pose2D(x=pos[0], y=pos[1])
    
    def callback_pose(self, data):
        """! Callback of the Pose Control node feedback topic.
        
        Sets the value of the pose_reached flag to false when it is true and 
        the message too, indicating that the robot's second position has been 
        reached.
        @param data std_msgs/Bool Pose Control node feedback
        """
        if not self.global_orientation_done and not self.pose_reached and data.data: 
            self.pose_reached = True
            rospy.loginfo('Meassuring position 2')
            self.second = self.measure_position()
            self.compute_global_orientation()


    def publish_tf_static(self, angle):
        """! Converts the angle in euler coordinates to quaternion 
        and publishes the world->odom transform (z axis rotation) to
        the static TF topic.

        @param angle Angle of the robot's orientation meassured from
        the north direction.
        """
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

        rospy.loginfo('Publishing static TF, angle: ' + str(angle))
        broadcaster.sendTransform(static_transformStamped)

    def callback_global_orientation_do(self, data):
        """! Callback of the command topic to perform the global orientation 
        estimation procedure.
        
        Masures the first position from GPS, then orders the robot to 
        move to the second position. As soon as it's reached, measures
        the second position, calculates the orientation of the robot
        and publishes it.

        @param data std_msgs/Empty message.
        """
        if not self.global_orientation_done:
            rospy.loginfo('Meassuring position 1')
            self.first = self.measure_position()
            self.pose_reached = False
            self.pub.publish(Pose2D(x=10.0, y=0.0, theta=0.0))
            
            # rate = rospy.Rate(1) # 0.1hz
            # while not self.pose_reached:
            #     rate.sleep()
            # rospy.loginfo('Meassuring position 2')
            # second = self.measure_position()
            # rospy.logdebug('First measurement: ' + str(first))
            # rospy.logdebug('Second measurement: ' + str(second))
            # north_angle = (arctan2(second.x - first.x, second.y - first.y) + pi) % (2*pi)
            # self.publish_tf_static(north_angle)
            # self.pub_global.publish(Bool(data=True))

    def callback_global_orientation_reset(self, data):
        self.publish_tf_static(0.0)
        self.global_orientation_done = False
    
    def compute_global_orientation(self):
        rospy.logdebug('First measurement: ' + str(self.first))
        rospy.logdebug('Second measurement: ' + str(self.second))
        north_angle = (arctan2(self.second.x - self.first.x, self.second.y - self.first.y) + pi) % (2*pi)
        self.publish_tf_static(north_angle)
        self.pub_global.publish(Bool(data=True))
        self.global_orientation_done = True



if __name__ == '__main__':
    try:
        rospy.init_node('gps_processing', anonymous=True, log_level=rospy.INFO)
        GPSProcessing()
    except rospy.ROSInterruptException:
        pass