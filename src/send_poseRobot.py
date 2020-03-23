#! /usr/bin/env python

import rospy
import time
from random import seed
from random import random
from std_msgs.msg import Header
from semantic_mapping.msg import SemanticObjects
from semantic_mapping.msg import SemanticObject
import geometry_msgs.msg
import math

def generatePose():
    seed(1)
    rospy.init_node('pose_Generator', anonymous=True)
    rate = rospy.Rate(10)

    pub = rospy.Publisher('poseRobot', geometry_msgs.msg.Vector3, queue_size=10)
    current_milli_time = lambda: int(round(time.time()*1000))
    time1 = current_milli_time()
    ang = 0
    while not rospy.is_shutdown():
        time2 = current_milli_time()
        if (time2 - time1 > 5000):
            msg = geometry_msgs.msg.Vector3(5,5,(ang*math.pi/180))
            pub.publish(msg)
            ang = ang + 1
            if(ang >= 360):
                ang = ang - 360
            time1 = current_milli_time()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        generatePose()
    except rospy.ROSInterruptException:
        pass