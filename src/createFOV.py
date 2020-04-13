#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
import geometry_msgs.msg
from room_categorization.msg import bestObjectInfo
from semantic_mapping.msg import SemanticObjects
from room_categorization.msg import debugInfo
from semantic_mapping.msg import SemanticRoom
import sys

class objectsInFOV(object):
    
    def __init__(self):

        # Variables de control de codigo
        self._debug = True
        self._newObjectFOVList = False
        self._publishRate = 10

        # Parametros del algoritmo de percepcion activa
        self._categoriaMinima = 2
        self._angle_offset = 0

        # Publishers
        self._pub = rospy.Publisher('bestObjectInfo', bestObjectInfo, queue_size=10)

        if self._debug:
            self._pubDebug = rospy.Publisher('consoleDebugInfo', debugInfo, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('semantic_mapping/object_in_room', SemanticObjects, self._newObjectList_callback)
        rospy.Subscriber("poseRobot", geometry_msgs.msg.Vector3, self._newRobotPose_callback)
        rospy.Subscriber("semantic_mapping/room_scores", SemanticRoom, self._newRoom_callback)
        
        # Parametros de los sensores
        self._nCameras = 4              # Numero de camaras
        self._height = 240              # Alto de la imagen de una camara
        self._width = 240               # Ancho de la imagen de una camara
        self._FOVcamera = 180           # Campo de vision horizontal
        self._FOVcameraVertical = 58    # Campo de vision vertical
        self._distZcamera = 1.045       # Distancia entre el suelo y la camara
        
        # Informacion de la localizacion del robot
        self._poseRobot = [1,2,3,80]    # Pose del robot (x,y,z,orientacion(grados))
        self._currentRoom = 'NoValid'   # Habitacion actual del robot

        # Informacion del FOV del robot
        self._angmax = None             # Angulo maximo del FOV horizontal del robot
        self._angmin = None             # Angulo minimo del FOV horizontal del robot
        self._angminc = None            # Correccion del FOV horizontal

        # Constantes de interes
        self._deg2rad = math.pi/180     # Paso de grados a radianes
        self._rad2deg = 180/math.pi     # Paso de radianes a grados
        self._angle2px = self._nCameras*self._width/self._FOVcamera
        self._px2angle = self._FOVcamera/(self._nCameras*self._width)
       
        # Informacion del entorno
        self._mapa = {}                 # Mapa de los objetos en la habitacion actual  
        self.objIn = {}                 # Objetos dentro del FOV del robot

        # Variables de control
        self._firstRoomExploration = False
        self._current_milli_time = lambda: int(round(time.time()*1000))
    
    def _getFOV(self):
        # Obtengo el angulo minimo del FOV
        self._angmin = (self._poseRobot[3] - self._FOVcamera / 2 + 22.5)*self._deg2rad  # Angulo minimo del FOV

        # Proceso el FOV para que siempre se cumpla que angmax >= angmin. Para ello, lo que se hace es
        # girar todos los angulos para que angmin valga 0 grados, de tal forma que se cumple siempre la condicion
        # propuesta.
        self._angmin = self.angle_range0to2pi(self._angmin) # Angmin entre 0 y 2pi              
        self._angminc = 2*math.pi - self._angmin # Correccion necesaria para situar angmin en 0 grados

        # Angulo del FOV corregido
        self._angmax = self._FOVcamera*self._deg2rad                        # Angmax = FOV
        self._angmin = self.angle_range0to2pi(self._angmin + self._angminc) # Angmin = 0
        
    def _createList_objectsInFOV(self):
        self.objIn.clear()

        for obj in self._mapa:
            # Compruebo si el objeto esta en la misma hab que el robot
            if ((self._mapa[obj][0] == self._currentRoom) and (self._mapa[obj][2] >= self._categoriaMinima)):
                [isIn, angle] = self._checker_objectinFOV(obj)
                if(isIn == True):
                    self.objIn[obj] = self._mapa[obj] + (angle,)

        self._newObjectFOVList = True
        
    def _checker_objectinFOV(self, objeto):
        # Comprobacion de si un objeto esta dentro del campo de vision del robot o no.
        angle = math.atan2((self._mapa[objeto][5]-self._poseRobot[1]),(self._mapa[objeto][4]-self._poseRobot[0]))
        angle = self.angle_range0to2pi(angle + self._angminc + 30*self._deg2rad)
        if (angle >= (self._angmin + self._angle_offset*self._deg2rad) and angle <= (self._angmax - self._angle_offset*self._deg2rad)):
            # Objeto dentro del campo de vision horizontal... pero, esta dentro del campo de vision vertical?
            distObjetoRobot = math.sqrt((self._poseRobot[0]-self._mapa[objeto][4])**2 + (self._poseRobot[0]-self._mapa[objeto][5])**2)
            hZ = distObjetoRobot*math.tan(self._FOVcameraVertical/2*self._deg2rad)
            hZmin = self._distZcamera - hZ
            hZmax = self._distZcamera + hZ
            if(self._mapa[objeto][6] <= hZmax and self._mapa[objeto][6] >= hZmin):
                # Esta dentro del campo de vision del robot
                return True, angle
            else:
                return False, angle
        else:
            return False, angle
        
    def run(self):

        # Variables de control
        exploration = False
        listaObjIn = []
        rate = rospy.Rate(self._publishRate)

        # Constantes
        #minPx = self._width/2
        #maxPx = self._nCameras*self._width - minPx
        minPx = 0
        maxPx = self._nCameras*self._width
        
        # Informacion sobre el mejor objeto
        bestObj = None

        while not rospy.is_shutdown():

            if (bool(self.objIn) and self._newObjectFOVList and not(self._firstRoomExploration)):

                exploration = False
                listaObjIn = self.objIn.keys()
                bestObj = sorted(self.objIn.items(), key = lambda x: x[1][2] + x[1][3], reverse = True)
                bestObj = bestObj[0]

                px = int(round((minPx - maxPx) * bestObj[1][7] / (self._angmax - self._angmin) + maxPx))

                msg = bestObjectInfo(bestObj[1][2], px)

                if self._debug:
                    debug_msg = debugInfo(self._currentRoom, exploration, bestObj[1][1], bestObj[1][0], geometry_msgs.msg.Vector3(bestObj[1][4], bestObj[1][5], bestObj[1][6]), px, listaObjIn)

            elif (not(bool(self.objIn)) or self._firstRoomExploration):

                exploration = True
                
                msg = bestObjectInfo(0, 0)

                if self._firstRoomExploration:
                    msg = bestObjectInfo(-1, 0)
                    self._firstRoomExploration = False

                if self._debug:
                    debug_msg = debugInfo(self._currentRoom, exploration, 'None', 'None', geometry_msgs.msg.Vector3(), 0, [])

            self._pub.publish(msg)

            if self._debug:
                self._pubDebug.publish(debug_msg)

            rate.sleep()


    def _newRobotPose_callback(self, data):
        self._poseRobot = [data.x,data.y,0.0,self.angle_range0to2pi(data.z)*self._rad2deg]
        self._getFOV()
        self._createList_objectsInFOV()

    def _newRoom_callback(self, data):
        if data.id != self._currentRoom:
            self._currentRoom = data.id
            #self._firstRoomExploration = True

    def _newObjectList_callback(self, data):
        self._mapa.clear()
        for obj in data.semanticObjects:
            self._mapa[obj.id] = (obj.idRoom, obj.type)
            # Objetos que dan gran informacion sobre la habitacion. Categoria 4
            if(obj.type == 'microwave' or obj.type == 'oven' or obj.type == 'toaster' or obj.type == 'bed' or obj.type == 'toilet'):
                self._mapa[obj.id] = self._mapa[obj.id] + (4,)
            # Objetos que dan algo de informacion sobre la habitacion. Categoria 3
            elif(obj.type == 'dining table' or obj.type == 'sink'):
                self._mapa[obj.id] = self._mapa[obj.id] + (3,)
            # Objetos que dan poca informacion sobre la habitacion. Categoria 2
            elif(obj.type == 'tv' or obj.type == 'bench' or obj.type == 'couch'):
                self._mapa[obj.id] = self._mapa[obj.id] + (2,)
            # Objetos que dan muy poca informacion sobre la habitacion. Categoria 1
            elif(obj.type == 'chair'):
                self._mapa[obj.id] = self._mapa[obj.id] + (1,)
            else:
                self._mapa[obj.id] = self._mapa[obj.id] + (0,)
            self._mapa[obj.id] = self._mapa[obj.id] + (obj.score, obj.pose.position.x, obj.pose.position.z, obj.pose.position.y,)
            #if(self._currentRoom != obj.idRoom):
            #    self._currentRoom = obj.idRoom
        self._getFOV()
        self._createList_objectsInFOV()

    @staticmethod
    def angle_range0to2pi(angle):
        while(angle >= 2*math.pi or angle < 0):
            if(angle >= 2*math.pi):
                angle = angle - 2*math.pi
            elif(angle < 0):
                angle = 2*math.pi + angle
        return angle

def main():
    rospy.init_node('objectsInFOV', anonymous=True)
    node = objectsInFOV()
    node.run()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
