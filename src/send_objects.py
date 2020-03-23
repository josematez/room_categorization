#! /usr/bin/env python

import rospy
import time
from random import seed
from random import random
from std_msgs.msg import Header
from semantic_mapping.msg import SemanticObjects
from semantic_mapping.msg import SemanticObject
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud

def generateObjects():
    seed(1)
    rospy.init_node('pose_Generator', anonymous=True)
    rate = rospy.Rate(10)

    pub = rospy.Publisher('semantic_mapping/object_in_room', SemanticObjects, queue_size=10)
    current_milli_time = lambda: int(round(time.time()*1000))
    time1 = current_milli_time()
    ang = 0
    while not rospy.is_shutdown():
        time2 = current_milli_time()
        if (time2 - time1 > 1000):
            obj1 = SemanticObject('Microwave1', 'Kitchen1', 'http://mapir.isa.uma.es/Microwave', 0.98745, 
            geometry_msgs.msg.Pose(geometry_msgs.msg.Point(2.5, 8.1, 2.5),geometry_msgs.msg.Quaternion(1.0, 1.0, 1.0, 1.0)), 
            geometry_msgs.msg.Vector3(1,1,1), PointCloud())
            obj2 = SemanticObject('DiningTable1', 'Kitchen1', 'http://mapir.isa.uma.es/Dining_Table', 0.88745, 
            geometry_msgs.msg.Pose(geometry_msgs.msg.Point(8.3, 8.2, 1.0),geometry_msgs.msg.Quaternion(1.0, 1.0, 1.0, 1.0)), 
            geometry_msgs.msg.Vector3(1,1,1), PointCloud())
            obj3 = SemanticObject('Chair1', 'Kitchen1', 'http://mapir.isa.uma.es/Chair', 0.78745, 
            geometry_msgs.msg.Pose(geometry_msgs.msg.Point(9.8, 2.1, 1.0),geometry_msgs.msg.Quaternion(1.0, 1.0, 1.0, 1.0)), 
            geometry_msgs.msg.Vector3(1,1,1), PointCloud())
            msg = SemanticObjects(Header(), [obj1, obj2, obj3])
            pub.publish(msg)
            time1 = current_milli_time()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        generateObjects()
    except rospy.ROSInterruptException:
        pass