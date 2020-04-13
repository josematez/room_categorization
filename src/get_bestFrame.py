#!/usr/bin/env python

import rospy
import math
import time
import cv2
import tf
from tf.transformations import quaternion_from_euler
import numpy as np
from room_categorization.msg import bestObjectInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rosgraph_msgs.msg import Clock

class bestSubimage(object):

    def __init__(self):

        self._debug = False
        
        self._bridge = CvBridge()

        # Publishers
        self._pubRGB = rospy.Publisher('bestImage_RGB', Image, queue_size=10)
        self._pubD = rospy.Publisher('bestImage_D', Image, queue_size=10)


        # Subscribers
        rospy.Subscriber("panoramicImage_RGB", Image, self._newPanoramicRGB_callback)
        rospy.Subscriber("panoramicImage_D", Image, self._newPanoramicD_callback)
        rospy.Subscriber("bestObjectInfo", bestObjectInfo, self._newObject_callback)
        rospy.Subscriber("clock", Clock, self._rosbagClock_callback)

        # Variables of interest

        # Publicar tf virtual
        self._broadcastTF = tf.TransformBroadcaster()

        # Parametros de los sensores
        self._nCameras = 4      # Numero de camaras
        self._height = 240      # Alto de la imagen de una camara
        self._width = 240       # Ancho de la imagen de una camara
        self._vPan = 15         # Velocidad de rotacion horizontal en grados/seg.
        self._FOVcamera = 180   # alpha en grados
        self._vPanPx = self._vPan*self._nCameras*self._width/self._FOVcamera    # Velocidad de rotacion horizontal en px/seg.

        # Images
        self._last_panoramic = None
        self._last_panoramicD = None
        self._last_subimg = None
        self._last_subimgD = None

        # Variables de interes
        self._last_xCenter = self._nCameras*self._width/2
        self._last_xCenterDestino = None

        # Variables de control
        self._startedRGB = False
        self._startedD = False
        self._currentTime_secs = 0
        self._currentTime_nsecs = 0
        self._publishRate = 4
        self._exploration = True
        self._newObject = True
        self._firstExploration = False

        # Variables para debug
        self._current_milli_time = lambda: int(round(time.time()*1000))
        self._lastTime = self._current_milli_time()

    def _get_subImage_pxCenter(self, xCenter):
        self._last_xCenter = int(round(xCenter))
        # Dado el centro de la imagen, obtengo la subimagen resultante a partir de la panoramica.
        if(self._last_xCenter > self._width/2 and self._last_xCenter < 3*self._width+self._width/2):
            self._last_subimg = self._last_panoramic[:,self._last_xCenter-self._width/2:self._last_xCenter+(self._width/2-1)]
            self._last_subimgD = self._last_panoramicD[:,self._last_xCenter-self._width/2:self._last_xCenter+(self._width/2-1)]
        elif(self._last_xCenter <= self._width/2):
            self._last_subimg = self._last_panoramic[:,0:self._width-1]
            self._last_subimgD = self._last_panoramicD[:,0:self._width-1]
        else:
            self._last_subimg = self._last_panoramic[:,-(self._width-1):]
            self._last_subimgD = self._last_panoramicD[:,-(self._width-1):]

    def _get_virtualTF(self):
        if(self._last_xCenter > self._width/2 and self._last_xCenter < 3*self._width+self._width/2):
            theta = -135*(self._last_xCenter - 840)/720 - 90
        elif(self._last_xCenter <= self._width/2):
            theta = 45
        else:
            theta = -90
        x = -5.062*pow(10,-6)*theta*theta + 1.111*pow(10,-6)*theta + 0.2812
        y = 2.03*pow(10,-6)*theta*theta + 0.00067*theta - 0.00255
        # self._lastTime2 = self._current_milli_time()
        #if(self._lastTime2 - self._lastTime > 200):
        #    self._lastTime = self._lastTime2
        self._broadcastTF.sendTransform((x,y,1.045), quaternion_from_euler(math.pi/2,0,(theta-90)*math.pi/180), rospy.Time(self._currentTime_secs, self._currentTime_nsecs), "RGBD_virtual","base_link")

    def _rotate(self, t_inicio, t_total, pxInicio, pxFin):
        t_actual = self._current_milli_time()

        if (t_actual - t_inicio < t_total):
            pxNext = (pxFin - pxInicio)*(t_actual - t_inicio)//t_total + pxInicio
            finished = False
        else:
            pxNext = pxFin
            finished = True
        
        return pxNext, finished

    def _newPanoramicRGB_callback(self,data):
        self._last_panoramic = self._bridge.imgmsg_to_cv2(data, 'rgb8')
        self._startedRGB = True

    def _newPanoramicD_callback(self,data):
        self._last_panoramicD = self._bridge.imgmsg_to_cv2(data, 'passthrough')
        self._startedD = True

    def _newObject_callback(self,data):
        if self._startedRGB and self._startedD:
            self._last_xCenterDestino = data.xCenter
            if(data.category == 0):
                self._exploration = True
            elif(data.category == -1):
                self._firstExploration = True
                self._exploration = False
            else:
                self._exploration = False
            self._newObject = True
    
    def _rosbagClock_callback(self,data):
        self._currentTime_secs = data.clock.secs
        self._currentTime_nsecs = data.clock.nsecs

    def run(self):

        rate = rospy.Rate(self._publishRate)

        # Variables de interes
        lim_left = self._width//2
        lim_right = (self._nCameras-0.5)*self._width
        lim_center = self._nCameras*self._width//2

        # Variables control movimiento
        direccion = None
        last_move = None
        t_inicio = None
        t_total = None
        pxInicio = None
        pxFin = None
        datos_OK = False
        finalizado = True
        firstExploration_left = False
        firstExploration_right = False

        while not rospy.is_shutdown():
            if self._startedRGB and self._startedD:

                if self._exploration and not self._firstExploration:
                    if not datos_OK and finalizado:

                        # Ir hacia la izquierda
                        if (((self._last_xCenter <= lim_center) and (self._last_xCenter > lim_left)) or (self._last_xCenter == lim_right) or (direccion == 'right')):
                            direccion = 'left'
                            pxFin = lim_left

                        # Ir hacia la derecha
                        elif (((self._last_xCenter > lim_center) and (self._last_xCenter < lim_right)) or (self._last_xCenter == lim_left) or (direccion == 'left')):
                            direccion = 'right'
                            pxFin = lim_right

                        pxInicio = self._last_xCenter

                        t_total = 1000*abs(self._last_xCenter - pxFin)/self._vPanPx
                        t_inicio = self._current_milli_time()

                        datos_OK = True
                        last_move = None
                    
                    self._last_xCenter, finalizado = self._rotate(t_inicio, t_total, pxInicio, pxFin)

                    if self._last_xCenter == lim_left or self._last_xCenter == lim_right:
                        datos_OK = False

                elif self._firstExploration:
                
                    print("First Exploration")
                    print("first_left: {}, first_right: {}, datos_OK: {}" .format(firstExploration_left, firstExploration_right, datos_OK))
                    if not firstExploration_left and not datos_OK:
                        direccion = 'left'
                        pxFin = lim_left
                        pxInicio = self._last_xCenter
                        t_total = 1000*abs(self._last_xCenter - pxFin)/self._vPanPx
                        t_inicio = self._current_milli_time()
                        datos_OK = True
                        #print("First Exploration - Left Finished")

                    elif firstExploration_left and not firstExploration_right and not datos_OK:
                        direccion = 'right'
                        pxFin = lim_right
                        pxInicio = self._last_xCenter
                        t_total = 1000*abs(self._last_xCenter - pxFin)/self._vPanPx
                        t_inicio = self._current_milli_time()
                        datos_OK = True
                        print("Hola")

                    self._last_xCenter, finalizado = self._rotate(t_inicio, t_total, pxInicio, pxFin)
                    
                    if self._last_xCenter == lim_left and finalizado:
                        print("First Exploration - Left Finished")
                        firstExploration_left = True
                        datos_OK = False
                    elif self._last_xCenter == lim_right and finalizado:
                        print("First Exploration - Right Finished")
                        firstExploration_right = True
                        datos_OK = False

                    if firstExploration_left and firstExploration_right:
                        print("Error 3")
                        self._firstExploration = False
                        firstExploration_left = False
                        firstExploration_right = False
                        print("First Exploration - Finished")

                else:

                    if self._last_xCenter > self._last_xCenterDestino:
                        direccion = 'left'
                    elif self._last_xCenter < self._last_xCenterDestino:
                        direccion = 'right'
                    else:
                        direccion = 'stop'

                    if self._newObject and (direccion != last_move):
                        t_inicio = self._current_milli_time()
                        self._newObject = False
                        last_move = direccion
                        datos_OK = False

                    t_total = 1000*abs(self._last_xCenter - self._last_xCenterDestino)/self._vPanPx

                    self._last_xCenter, finalizado = self._rotate(t_inicio, t_total, self._last_xCenter, self._last_xCenterDestino)
                
                self._get_subImage_pxCenter(self._last_xCenter)
                img2pubRGB = self._bridge.cv2_to_imgmsg(self._last_subimg, 'rgb8')
                img2pubD = self._bridge.cv2_to_imgmsg(self._last_subimgD, 'mono16')
                self._get_virtualTF()
                img2pubRGB.header.frame_id = 'RGBD_virtual'
                img2pubD.header.frame_id = 'RGBD_virtual'
                self._pubRGB.publish(img2pubRGB)         
                self._pubD.publish(img2pubD)

            rate.sleep()


def main():
    rospy.init_node('get_bestSubimage', anonymous=True)
    node = bestSubimage()
    node.run()    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
