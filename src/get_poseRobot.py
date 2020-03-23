#!/usr/bin/env python  
import rospy
import math
import tf2_ros
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('poseRobot')

    # Publishers
    pub = rospy.Publisher('poseRobot', geometry_msgs.msg.Vector3, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    trans = None
    while not rospy.is_shutdown():
        try:
            last_trans = trans
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        if(last_trans != trans):
            quaternion = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
            rpy = euler_from_quaternion(quaternion)
            # Envia (x,y, theta), ya que z es fijo, y theta es la orientacion
            msg = geometry_msgs.msg.Vector3(trans.transform.translation.x, trans.transform.translation.y, rpy[2])
            pub.publish(msg)

        rate.sleep()
