#!/usr/bin/env python3
#import roslib 
#roslib.load_manifest('imu')

import rospy
import lsm6ds0 
import time 
import computation
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from time import time
from tf.transformations import quaternion_from_euler 



def publisher():
    # define the actions the publisher will make
    pub = rospy.Publisher('imu',Imu,queue_size =10)

    #initalize the publishing node 
    rospy.init_node('ros_imu',anonymous = True)

    # define how many times per second
    # will the data be published
    # let's say 10 times/second or 10Hz
    rate = rospy.Rate(10)


    msg = Imu()
    # to keep publishing as long as the core is running
    while not rospy.is_shutdown():
        
        msg.header.stamp = rospy.get_rostime()
        
        
        #read acc and gyro
        [xa,ya,za] = lsm6ds0.lsm6ds0_get_acc()
        [rollv,pitchv,yawv] = lsm6ds0.lsm6sl_get_gyro()

        # caculate roll,pitch and yaw 
        roll = computation.compute_filtered_roll([xa,ya,za])
        pitch =computation.compute_filtered_pitch([xa,ya,za])
        yaw = 0

        #construct linear acceleration message 
        msg.linear_acceleration.x = xa
        msg.linear_acceleration.y = ya
        msg.linear_acceleration.z = za

        #conctruct angular velocity message
        msg.angular_velocity.x = rollv 
        msg.angular_velocity.y = pitchv
        msg.angular_velocity.z = yawv
        
        
        
        #construct quaternion orientation 
        q = quaternion_from_euler(roll,pitch,yaw)

        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]

        pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        lsm6ds0.lsm6dsl_init() 
        publisher()
    except rospy.ROSInterruptException:
        pass