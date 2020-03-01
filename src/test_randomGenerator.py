#! /usr/bin/env python

import rospy
import time
from random import seed
from random import random
from std_msgs.msg import Float64
from std_msgs.msg import Header
#from room_categorization.msg import roomScores
#from room_categorization.msg import semanticRoom
from semantic_mapping.msg import SemanticRoom
from semantic_mapping.msg import SemanticRoomScore

def generateRandomNumber():
    seed(1)
    rospy.init_node('randomNumber_generator', anonymous=True)
    rate = rospy.Rate(10)

    # pub = rospy.Publisher("room_scores", Float64, queue_size=10)
    pub = rospy.Publisher("semantic_mapping/room_scores", SemanticRoom, queue_size=10)
    current_milli_time = lambda: int(round(time.time()*1000))
    time1 = current_milli_time()
    while not rospy.is_shutdown():
        time2 = current_milli_time()
        if (time2 - time1 > 5000):
            msg1 = SemanticRoomScore('http://mapir.isa.uma.es/Kitchen', random())
            msg2 = SemanticRoomScore('http://mapir.isa.uma.es/Bedroom', random())
            msg3 = SemanticRoomScore('http://mapir.isa.uma.es/Living_Room', random())
            msg4 = SemanticRoomScore('http://mapir.isa.uma.es/Dressing_Room', random())
            msg5 = SemanticRoomScore('http://mapir.isa.uma.es/Bathroom', random())
            msg = SemanticRoom(Header(),'Kitchen', [msg1, msg2, msg3, msg4, msg5])
            #msg = [random(), random(), random(), random(), random()]
            pub.publish(msg)
            #print("Score: %f", msg)
            print(msg)
            time1 = current_milli_time()
        rate.sleep()
    

if __name__ == '__main__':
    try:
        generateRandomNumber()
    except rospy.ROSInterruptException:
        pass