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

    def _animate(self,i):
        if self._nHabitaciones != 0:
            print("pintando!")
            self._ax1.clear()
            plt.title('Room scores evolution - Camera 1')
            plt.xlabel('Detections')
            plt.ylabel('Room Scores')
            self._ax1.plot(self._tempDetections, self._kitchen, label = 'Kitchen')
            self._ax1.plot(self._tempDetections, self._bathroom, label = 'Bathroom')
            self._ax1.plot(self._tempDetections, self._bedroom, label = 'Bedroom')
            self._ax1.plot(self._tempDetections, self._dressingroom, label = 'Dressing_Room')
            self._ax1.plot(self._tempDetections, self._livingroom, label = 'Living_Room')
            plt.legend(loc = 'upper right')
        if self._newHab:
            # ESTO ESTA MAL IMPLEMENTADO, ARREGLAR PARA QUE APAREZCA UN TEXTO CADA VEZ QUE CAMBIE DE HABITACION
            self._newHab = False
            plt.text(self._nDetections, 0.9, self._last_hab)
            

    def run(self):
        rate = rospy.Rate(10000)

        while not rospy.is_shutdown():
            a = 1
            a = a + 1
            ani = animation.FuncAnimation(self._fig, self._animate, interval=10, repeat=False)
            plt.show()

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

def main():
    rospy.init_node('plotScores', anonymous=True)
    node = room_scores()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
