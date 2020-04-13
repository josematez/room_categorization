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
        # Publishers
        self._pub = rospy.Publisher('bestObjectInfo', bestObjectInfo, queue_size=10)
        self._pubDebug = rospy.Publisher('consoleDebugInfo', debugInfo, queue_size=10)
        
        # Subscribers
        # /objects_in_room - Recibe lista con msg 'SemanticObjects':
            # - string: id
            # - string: id_room
            # - string: type
            # - float64: score
            # - geometry_msgs/PoseStamped pose
            # - geometry_msgs/Vector3 scale
        rospy.Subscriber('semantic_mapping/object_in_room', SemanticObjects, self._newObjectList_callback)
        # /poseRobot - Recibo la pose del robot:
        rospy.Subscriber("poseRobot", geometry_msgs.msg.Vector3, self._newRobotPose_callback)
            
        # Variables of interest
        
        # Parametros de los sensores
        self._nCameras = 4 # Numero de camaras
        self._height = 240 # Alto de la imagen de una camara
        self._width = 240  # Ancho de la imagen de una camara
        self._vPan = 0    # Velocidad de rotacion horizontal en grados/seg.
        self._FOVcamera = 180 # Campo de vision horizontal
        self._FOVcameraVertical = 58 # Campo de vision vertical
        self._distZcamera = 1.045   # Distancia entre el suelo y la camara
        
        #Informacion del robot
        self._poseRobot = [1,2,3,80] # Pose del robot (x,y,z,orientacion(grados))
        self._angmax = None           # Angulo maximo del FOV horizontal del robot
        self._angmin = None           # Angulo minimo del FOV horizontal del robot
        self._angminc = None          # Correccion del FOV horizontal
        self._angmax_plot = None      # Angulo maximo del FOV sin procesar del robot
        self._angmin_plot = None      # Angulo minimo del FOV sin procesar del robot
        self._currentRoom = 'NoValid' # Habitacion actual del robot
        
        # Variables de control 
        self._newObjectFOVList = False

        # Constantes de interes
        self._deg2rad = math.pi/180  # Paso de grados a radianes
        self._rad2deg = 180/math.pi  # Paso de radianes a grados
       
        # Informacion del entorno
        #self._mapa = {'DiningTable1': ('Kitchen1','Dining table', 3, 0.7892324, 0.4, 2.0,0.0),  #Se almacenan los objetos reconocidos
        #   'DiningTable2': ('Kitchen1','Dining table', 3, 0.9828723, 5.5, 9.0,0.0),
        #   'DiningTable3': ('Kitchen2','Dining table', 3, 0.7609432, -3.4, -2.0,0.0),
        #   'DiningTable4': ('Kitchen2','Dining table', 3, 0.8892324, -0.4, -2.0,0.0),
        #   'Chair1': ('Kitchen1','Chair', 1, 0.6792324, 5.4, 2.0,0.0),                
        #   'Chair2': ('Kitchen1','Chair', 1, 0.9792324, 7.4, 2.0,0.0),
        #   'Microwave1': ('Kitchen1','Microwave', 4, 0.7534982, 5.4, 6.5,0.0),
        #   'Bed1': ('Bedroom1','Bed', 4, 0.8792324, -2.0, 2.0,0.0)}
        self._mapa = {}
        self.objIn = {}                    # Objetos dentro del FOV del robot
    
    def _getFOV(self):
        # Obtengo el FOV
        # El +22.5 es debido a que el FOV no esta centrado
        self._angmax = (self._poseRobot[3] + self._FOVcamera / 2 + 22.5)*self._deg2rad  # Angulo maximo del FOV
        self._angmin = (self._poseRobot[3] - self._FOVcamera / 2 + 22.5)*self._deg2rad  # Angulo minimo del FOV
        # Proceso el FOV para que siempre se cumpla que angmax >= angmin. Para ello, lo que se hace es
        # girar todos los angulos para que angmin valga 0 grados, de tal forma que se cumple siempre la condicion
        # propuesta.
        self._angmax = self.angle_range0to2pi(self._angmax) # Angmax entre 0 y 2pi
        self._angmin = self.angle_range0to2pi(self._angmin) # Angmin entre 0 y 2pi
        self._angmax_plot = self._angmax # Almaceno el angmax para plotear antes de procesarlo
        self._angmin_plot = self._angmin # Almaceno el angmin para plotear antes de procesarlo                  
        self._angminc = 2*math.pi - self._angmin # Correccion necesaria para situar angmin en 0 grados
        self._angmax = self.angle_range0to2pi(self._angmax + self._angminc) # Correccion de angmax
        self._angmin = self.angle_range0to2pi(self._angmin + self._angminc) # Correccion de angmin
        
    def _createList_objectsInFOV(self):
        self.objIn.clear()
        for obj in self._mapa:
            if (self._mapa[obj][0] == self._currentRoom): # Compruebo si el objeto esta en la misma hab que el robot
                [isIn, angle] = self._checker_objectinFOV(obj)
                if(isIn == True):
                    self.objIn[obj] = self._mapa[obj] + (angle,)
        print("[MAPA]: {}" .format(self._mapa))
        print("--------------------------")
        print("[MAPA OBJIN]: {}" .format(self.objIn))
        self._newObjectFOVList = True

        #print("[OBJETOS MAPA]: " + str(self._mapa))
        #print("[POSE ROBOT] x: {}, y: {}, theta: {}" .format(self._poseRobot[0], self._poseRobot[1], self._poseRobot[3]))
        #print("[OBJETOS FOV]: ################")
        #for obj in self.objIn:
        #    print("{} (Cat: {}) en x: {}, y: {}, z: {}" .format(self.objIn[obj][1], self.objIn[obj][2], self.objIn[obj][4], self.objIn[obj][5], self.objIn[obj][6]))
        #print("#######################")

    # Funcion para debug. No utilizar en modo online ya que tarda mucho en pintar, por lo que las variables cambian y deja de tener sentido el plot
    def _plotScenario(self):
        # Calculo FOV para plotear
        xplotmax = [5*math.cos(self._angmax_plot) + self._poseRobot[0], self._poseRobot[0]]
        yplotmax = [5*math.sin(self._angmax_plot) + self._poseRobot[1], self._poseRobot[1]]
        xplotmin = [5*math.cos(self._angmin_plot) + self._poseRobot[0], self._poseRobot[0]]
        yplotmin = [5*math.sin(self._angmin_plot) + self._poseRobot[1], self._poseRobot[1]]
        plt.figure(1)
        plt.clf()
        plt.scatter(self._poseRobot[0],self._poseRobot[1])
        plt.plot(xplotmax,yplotmax,xplotmin,yplotmin)
        for obj in self._mapa:
            [isIn, angle] = self._checker_objectinFOV(obj)
            if isIn:
                plt.scatter(self._mapa[obj][4],self._mapa[obj][5],c=(0,1,0))
            else:
                plt.scatter(self._mapa[obj][4],self._mapa[obj][5],c=(1,0,0))
        plt.show(block = False)
        plt.pause(0.0001)
        
    def _checker_objectinFOV(self, objeto):
        # Comprobacion de si un objeto esta dentro del campo de vision del robot o no.
        angle = math.atan2((self._mapa[objeto][5]-self._poseRobot[1]),(self._mapa[objeto][4]-self._poseRobot[0]))
        angle = self.angle_range0to2pi(angle + self._angminc + 22.5*self._deg2rad)
        #if(self._currentRoom == self._mapa[objeto][0]):
        if (angle >= self._angmin and angle <= self._angmax):
            #self.objIn[objeto] = (self._mapa[objeto][0], self._mapa[objeto][1], angle)
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

    def _debugInfo(self, obj, ang, x, y, z):
        sys.stdout.write('\x1b[1A')
        sys.stdout.write('\x1b[2K')
        sys.stdout.write('\x1b[1A')
        sys.stdout.write('\x1b[2K')
        print("[POSE ROBOT]: x: {}, y: {}, theta: {}" .format(self._poseRobot[0], self._poseRobot[1], self._poseRobot[3]))
        print("Objeto: {}, angulo: {}, x: {}, y: {}, z: {}" .format(obj, ang*self._rad2deg, x, y, z))
        
    def run(self):
        catBestObject = None
        angleBestObject = None
        scoreBestObject = None
        bestObj = None
        exploration = True
        listaObjIn = []
        #last_bestObj = None
        while not rospy.is_shutdown():
            if(bool(self.objIn) and self._newObjectFOVList):
                exploration = False
                catBestObject = 0
                listaObjIn = []
                for obj in self.objIn:
                    if(self.objIn[obj][2] > catBestObject):
                        catBestObject = self.objIn[obj][2]
                        scoreBestObject = self.objIn[obj][3]
                        angleBestObject = self.objIn[obj][7]
                        bestObj = self.objIn[obj][1]
                        idBestObj = obj
                    elif(self.objIn[obj][2] == catBestObject and self.objIn[obj][3] > scoreBestObject):
                        catBestObject = self.objIn[obj][2]
                        scoreBestObject = self.objIn[obj][3]
                        angleBestObject = self.objIn[obj][7]
                        bestObj = self.objIn[obj][1]
                        idBestObj = obj
                    listaObjIn.append(self.objIn[obj][1])
                px = int(round(self._nCameras*self._width - (((angleBestObject - self._angmin)*self._nCameras*self._width) / (self._angmax - self._angmin))))
                msg = bestObjectInfo(catBestObject, px)
                self._pub.publish(msg)
                debug_msg = debugInfo(self._currentRoom, exploration, bestObj, self._mapa[idBestObj][0], geometry_msgs.msg.Vector3(self._mapa[idBestObj][4], self._mapa[idBestObj][5], self._mapa[idBestObj][6]), px, listaObjIn)
                self._pubDebug.publish(debug_msg)
                # self._debugInfo(bestObj, angleBestObject, self._mapa[idBestObj][4], self._mapa[idBestObj][5], self._mapa[idBestObj][6])
                self._newObjectFOVList = False
            elif(not(bool(self.objIn)) and (catBestObject != 0 or angleBestObject != 0 or scoreBestObject!= 0)):
                exploration = True
                bestObj = None
                catBestObject = 0
                angleBestObject = 0
                scoreBestObject = 0
                msg = bestObjectInfo(catBestObject, angleBestObject)
                self._pub.publish(msg)
                debug_msg = debugInfo(self._currentRoom, exploration, 'None', 'None', geometry_msgs.msg.Vector3(), 0, [])
                self._pubDebug.publish(debug_msg)
    
    def _newRobotPose_callback(self, data):
        self._poseRobot = [data.x,data.y,0.0,self.angle_range0to2pi(data.z)*self._rad2deg]
        self._getFOV()
        self._createList_objectsInFOV()
    
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
