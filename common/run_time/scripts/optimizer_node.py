#!/usr/bin/env python2.7

import roslib
import rospy
import sys

from optimizer import bleh

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rate = float(rospy.get_param("optimizer_node/rate", '-1.0'))
        dummy_val = float(rospy.get_param("optimizer_node/dummy_val", '-1.0'))

        if dummy_val > 0:
            print dummy_val
        else:
            print "can't receive param!"
        
        if rate:
            rospy.sleep(1/rate)
        else:
            rospy.sleep(1.0)
