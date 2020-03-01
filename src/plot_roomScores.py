#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from semantic_mapping.msg import SemanticRoom
#from room_categorization.msg import roomScores

class room_scores(object):

    def __init__(self):

        # Variables
        self._nDetections = 0
        self._nHabitaciones = 0
        self._last_hab = None
        self._newHab = False
        self._newData = False

        # Variables auxiliares de control

        # Scores de habitaciones
        self._kitchen = []
        self._bathroom = []
        self._livingroom = []
        self._dressingroom = []
        self._bedroom = []
        self._tempDetections = []

        # Grafica
        self._fig = plt.figure()
        self._ax1 = self._fig.add_subplot(1,1,1)

        #Subscribers
        rospy.Subscriber("semantic_mapping/room_scores", SemanticRoom, self._newScore)

    def run(self):
        rate = rospy.Rate(10000)
        #plt.ion()
        #plt.show()
        #plt.figure()
        while not rospy.is_shutdown(): 
            if self._newData == True:
                print("Ploteando...")
                plt.clf()
                plt.plot(self._tempDetections, self._kitchen, label = 'Kitchen')
                plt.plot(self._tempDetections, self._bathroom, label = 'Bathroom')
                plt.plot(self._tempDetections, self._bedroom, label = 'Bedroom')
                plt.plot(self._tempDetections, self._dressingroom, label = 'Dressing_Room')
                plt.plot(self._tempDetections, self._livingroom, label = 'Living_Room')
                plt.legend(loc = 'upper right')
                plt.show(block = False)
                plt.pause(0.0001)
                self._newData = False
                #plt.savefig('/home/matez/Downloads/Camera2scores.png')

    def _newScore(self,data):
        self._nDetections = self._nDetections + 1
        self._tempDetections.append(self._nDetections)
        #self._nHabitaciones = len(data.Probability)
        self._nHabitaciones = len(data.probabilities)
        if data.id != self._last_hab:
            self._newHab = True
            self._last_hab = data.id
        for hab in data.probabilities:
            if hab.type == 'http://mapir.isa.uma.es/Bathroom':
                self._bathroom.append(hab.score)
            elif hab.type == 'http://mapir.isa.uma.es/Kitchen':
                self._kitchen.append(hab.score)
            elif hab.type == 'http://mapir.isa.uma.es/Living_Room':
                self._livingroom.append(hab.score)
            elif hab.type == 'http://mapir.isa.uma.es/Dressing_Room':
                self._dressingroom.append(hab.score)
            elif hab.type == 'http://mapir.isa.uma.es/Bedroom':
                self._bedroom.append(hab.score)
        print("Deteccion %d" %self._nDetections)
        self._newData = True

def main():
    rospy.init_node('plotScores', anonymous=True)
    node = room_scores()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
