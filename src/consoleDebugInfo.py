#! /usr/bin/env python

import rospy
import math
import sys
from room_categorization.msg import debugInfo
from geometry_msgs.msg import Vector3

class info(object):

    def __init__(self):

        # Subscribers
        rospy.Subscriber("consoleDebugInfo", debugInfo, self._newInfo)
        rospy.Subscriber("poseRobot", Vector3, self._newRobotPose_callback)
        
        # Conversions
        self._rad2deg = 180/math.pi

        # Variables
        self._poseRobot = Vector3(0, 0, 0)
        self._currentRoom = None
        self._exploration = None
        self._idObject = None
        self._roomObject = None
        self._poseObject = Vector3(0, 0, 0)
        self._pxObject = None

        # Initialization
        print("Esto va a borrarse")
        print("Y esto tambien :(")

    def run(self):
        while not rospy.is_shutdown():
            sys.stdout.write('\x1b[1A')
            sys.stdout.write('\x1b[2K')
            sys.stdout.write('\x1b[1A')
            sys.stdout.write('\x1b[2K')
            print("[ROBOT] Room: {} - (x: {}, y: {}, theta: {})" .format(self._currentRoom, self._poseRobot.x, self._poseRobot.y, self._poseRobot.z))
            if self._exploration == True:
                print("[ACTIVE PERCEPTION] Explorando...")
            elif self._exploration == False:
                print("[ACTIVE PERCEPTION] Mirando a {} (x: {}, y: {}, z: {}) de {} en el px: {}" .format(self._idObject, self._poseObject.x, self._poseObject.y, self._poseObject.z, 
                self._roomObject, self._pxObject))
            else:
                print("[ACTIVE PERCEPTION] Stopped.")

    
    def _newInfo(self, data):
        self._currentRoom = data.currentRoom
        self._exploration = data.exploration
        self._idObject = data.idObject
        self._roomObject = data.objectRoom
        self._poseObject = data.poseObject
        self._pxObject = data.pxObject
    
    def _newRobotPose_callback(self, data):
        self._poseRobot = Vector3(data.x, data.y, self.angle_range0to2pi(data.z)*self._rad2deg)

    @staticmethod
    def angle_range0to2pi(angle):
        while(angle >= 2*math.pi or angle < 0):
            if(angle >= 2*math.pi):
                angle = angle - 2*math.pi
            elif(angle < 0):
                angle = 2*math.pi + angle
        return angle

def main():
    rospy.init_node('consoleDebugInfo', anonymous=True)
    node = info()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
