#!/usr/bin/env python3

from __future__ import print_function

import rospy
from sprinter_srvs.srv import GetOrientation

def printer_control_client():
    rospy.wait_for_service('/gps/get_sun_orientation')
    try:
        get_sun_pos = rospy.ServiceProxy('/gps/get_sun_orientation', GetOrientation)
        resp1 = get_sun_pos()
        print(resp1)
        return resp1.orientation
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Requesting")
    print("%s"%(printer_control_client()))