#!/usr/bin/env python3

## @package gps_processing
# Documentation for gps_processing node.

## @file gps_processing.py
# @brief Node in charge of determining the geographical position of the robot,
# its direction and the position of the sun at the given time and place.
#
# @section author_doxygen_example Author(s)
# - Created by Daniel Acuña on 28/05/2023.

# Imports
import rospy
from numpy import mean, arctan2, pi
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import NavSatFix, TimeReference
from geometry_msgs.msg import Pose2D, TransformStamped, Quaternion
from suncalc import get_position
from sprinter_srvs.srv import GetOrientation, GetOrientationResponse
from datetime import datetime, date
from collections import deque
from tf2_ros import StaticTransformBroadcaster
import tf

class GPSProcessing:
    """! The GPS processing class.

    Defines the GPS processing node class utilized.
    """
    def __init__(self):
        """! The GPSProcessing class initializer.
        @return  An instance of the GPSProcessing class initialized with the specified name.
        """
        ## Flag indicating if the robot has already moved to the second position.
        self.not_reached = True
        ## Queue of data taken from the GPS.
        self.q = deque()
        ## Size of ueue of data.
        self.qsize = 100
        ## Last GPS data collection time.
        self.GPS_time = datetime.now()
        ## Subscriber to gps/get_global_orientation/cmd topic.
        self.sub_global = rospy.Subscriber("/gps/get_global_orientation/cmd", Empty, self.callback_global)
        ## Subscriber to /gps/fix topic.
        self.sub_pos = rospy.Subscriber("/gps/fix", NavSatFix, self.callback_gps)
        ## Subscriber to /gps/time_ref topic.
        self.sub_pos_t = rospy.Subscriber("/time_reference", TimeReference, self.callback_gps_time)
        ## Subscriber to /target/pose/reached topic.
        self.sub_reached = rospy.Subscriber("/target/pose/reached", Bool, self.callback_reached)
        ## Publisher to /gps/get_global_orientation/done topic.
        self.pub_global = rospy.Publisher("/gps/get_global_orientation/done", Bool, queue_size=1)
        ## Publisher to /target/pose/cmd topic.
        self.pub = rospy.Publisher("/target/pose/cmd", Pose2D, queue_size=1)
        ## Service server on /gps/get_sun_orientation channel.
        self.sun_server = rospy.Service('/gps/get_sun_orientation', GetOrientation, self.handle_get_sun_orientation)
        rospy.spin()

    def handle_get_sun_orientation(self, req):
        """! Manager of sun orientation services. 
        
        Calculates the average of gps measurements and takes the last sampling time.
        With these parameters it calculates the azimuth angle and the height angle. 
        Finally convert these angles into quaternion coordinates.
        @return  Sun's position in Quaternion coordinates.
        """
        gps = self.measure_position()
        pos_sun = get_position(self.GPS_time, gps.y, gps.x)
        rospy.loginfo("Returning [%s, %s]"%(float(pos_sun['altitude']), float(pos_sun['azimuth'])+pi))
        quat = tf.transformations.quaternion_from_euler(
                   float(0),float(pos_sun['altitude']),float(pos_sun['azimuth']+pi))
        return GetOrientationResponse(Quaternion(*quat))
        
    def callback_gps_time(self, data):
        """! Callback of the GPS time. 
        
        Updates the last sample time each time a new sample appears.
        @param data New sample.
        """
        self.GPS_time = datetime.utcfromtimestamp(data.time_ref.to_sec())

    def callback_gps(self, data):
        """! Callback of the GPS position. 
        
        Adds a new sample to the end of the queue when it arrives. If the queue
        is full, remove the oldest sample.
        @param data new sample.
        """
        self.q.append([data.latitude, data.longitude])
        if(len(self.q) > self.qsize):
            rospy.logdebug('Ready')
            self.q.popleft()

    def measure_position(self):
        """! Computes the average of the samples in the queue.
        
        @return Pose2D object with the latitude in the x attribute and 
        longitude in the y attribute.
        """
        pos = mean(self.q, axis=0)
        return Pose2D(x=pos[0], y=pos[1])
    
    def callback_reached(self, data):
        """! Callback of /target/pose/reached topic.
        
        Sets the value of the not_research flag to false when it is true and 
        the message too, indicating that the robot's second position has been 
        reached.
        """
        if self.not_reached and data.data: self.not_reached = False

    def to_quaternion(self, angle):
        """! Computes the conversion from euler coordinates to the odom frame
        and publish the result in the static TF topic.

        @param angle Angle of robot's direction meassure from north direction.
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

        rospy.logdebug('Publishing static TF, angle: ' + str(angle))
        broadcaster.sendTransform(static_transformStamped)

    def callback_global(self, data):
        """! Callback of /gps/get_global_orientation/cmd topic.
        
        measure the first position and then wait for the robot to move to the 
        second position. Then it measures the second position, calculates the
        orientation of the robot and publishes it.

        @param data Void message.
        """
        rospy.logdebug('Meassuring position 1')
        first = self.measure_position()
        self.pub.publish(Pose2D(x=10.0, y=0.0, theta=0.0))
        self.not_reached = True
        rate = rospy.Rate(1) # 0.1hz
        while self.not_reached:
            rate.sleep()
        rospy.logdebug('Meassuring position 2')
        second = self.measure_position()
        rospy.logdebug('First measurement: ' + str(first))
        rospy.logdebug('Second measurement: ' + str(second))
        north_angle = arctan2(second.x - first.x, second.y - first.y)
        self.to_quaternion(north_angle)
        self.pub_global.publish(Bool(data=True))


if __name__ == '__main__':
    try:
        rospy.init_node('gps_processing', anonymous=True, log_level=rospy.DEBUG)
        GPSProcessing()
    except rospy.ROSInterruptException:
        pass