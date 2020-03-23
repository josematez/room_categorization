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
        self._vPan = 35        # Velocidad de rotacion horizontal en grados/seg.
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
        self._last_category = 0

        # Variables de control
        self._noObject = True
        self._startedRGB = False
        self._startedD = False
        self._currentTime_secs = 0
        self._currentTime_nsecs = 0

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
            self._last_subimg = self._last_panoramic[:,3*self._width-1:]
            self._last_subimgD = self._last_panoramicD[:,3*self._width-1:]

    def _get_virtualTF(self):
        if(self._last_xCenter > self._width/2 and self._last_xCenter < 3*self._width+self._width/2):
            #theta = 135*(self._last_xCenter - 120)/720 - 90
            theta = -135*(self._last_xCenter - 840)/720 - 90
        elif(self._last_xCenter <= self._width/2):
            #theta = -90
            theta = 45
        else:
            #theta = 45
            theta = -90
        if self._debug == True:
            print(self._last_xCenter <= self._width/2)
            print("[get_bestSubimage]: xCenter TF theta: " + str(self._last_xCenter))
            print("[get_bestSubimage]: virtual TF theta: " + str(theta))
        x = -5.062*pow(10,-6)*theta*theta + 1.111*pow(10,-6)*theta + 0.2812
        y = 2.03*pow(10,-6)*theta*theta + 0.00067*theta - 0.00255
        self._lastTime2 = self._current_milli_time()
        if(self._lastTime2 - self._lastTime > 500):
            self._lastTime = self._lastTime2
            self._broadcastTF.sendTransform((x,y,1.045), quaternion_from_euler(0,0,theta*math.pi/180), rospy.Time(self._currentTime_secs, self._currentTime_nsecs), "RGBD_virtual","base_link")
        
        
    def _rotation(self, pxInicio, pxFin, vel):
        if(pxInicio != pxFin):
            current_milli_time = lambda: int(round(time.time()*1000))
            T = 1000*abs(pxFin - pxInicio)/vel
            a = current_milli_time()
            b = current_milli_time()
            while (b - a < T):
                b = current_milli_time()
                xCenter = (pxFin - pxInicio)*(b-a)/T + pxInicio
                self._get_subImage_pxCenter(xCenter)
                img2pubRGB = self._bridge.cv2_to_imgmsg(self._last_subimg, 'rgb8')
                img2pubD = self._bridge.cv2_to_imgmsg(self._last_subimgD, 'mono16')
                self._get_virtualTF()
                img2pubRGB.header.frame_id = 'RGBD_virtual'
                img2pubD.header.frame_id = 'RGBD_virtual'
                self._pubRGB.publish(img2pubRGB)
                self._pubD.publish(img2pubD)

    def _rotation2(self, pxInicio, pxFin, vel):
        if(pxInicio != pxFin):
            current_milli_time = lambda: int(round(time.time()*1000))
            T = 1000*abs(pxFin - pxInicio)/vel
            a = current_milli_time()
            b = current_milli_time()
            while (b - a < T and self._noObject):
                b = current_milli_time()
                xCenter = (pxFin - pxInicio)*(b-a)/T + pxInicio
                self._get_subImage_pxCenter(xCenter)
                img2pubRGB = self._bridge.cv2_to_imgmsg(self._last_subimg, 'rgb8')
                img2pubD = self._bridge.cv2_to_imgmsg(self._last_subimgD, 'mono16')
                self._get_virtualTF()
                img2pubRGB.header.frame_id = 'RGBD_virtual'
                img2pubD.header.frame_id = 'RGBD_virtual'
                self._pubRGB.publish(img2pubRGB)
                self._pubD.publish(img2pubD)      
    
    def _exploration(self):
        if (self._last_xCenter <= self._nCameras*self._width/2):
            self._rotation2(self._last_xCenter, self._width/2, self._vPanPx)
            if self._noObject:
                self._rotation2(self._last_xCenter, (self._nCameras-1)*self._width + self._width/2, self._vPanPx)
        else:
            self._rotation2(self._last_xCenter, (self._nCameras-1)*self._width + self._width/2, self._vPanPx)
            if self._noObject:
                self._rotation2(self._last_xCenter, self._width/2, self._vPanPx)

    def _newPanoramicRGB_callback(self,data):
        self._last_panoramic = self._bridge.imgmsg_to_cv2(data, 'rgb8')
        self._startedRGB = True

    def _newPanoramicD_callback(self,data):
        self._last_panoramicD = self._bridge.imgmsg_to_cv2(data, 'passthrough')
        self._startedD = True

    def _newObject_callback(self,data):
        if self._startedRGB and self._startedD:
            self._last_xCenterDestino = data.xCenter
            self._last_category = data.category
            if(self._last_category == 0):
                self._noObject = True
                #print("Sin objeto de interes")
            else:
                self._noObject = False
                #print("Nuevo Objeto " + str(self._num) + "en " + str(self._last_xCenterDestino) + " y estoy en " + str(self._last_xCenter))
    
    def _rosbagClock_callback(self,data):
        self._currentTime_secs = data.clock.secs
        self._currentTime_nsecs = data.clock.nsecs

    def run(self):
        while not rospy.is_shutdown():
            if self._startedRGB and self._startedD:
                if(self._last_category != 0):
                    if self._debug == True:
                        print("[get_bestSubimage] Visualizando objeto importante!")
                    self._rotation(self._last_xCenter, self._last_xCenterDestino, self._vPanPx)
                else:
                    if self._debug == True:
                        print("[get_bestSubimage] Explorando!")
                    self._exploration()
                img2pubRGB = self._bridge.cv2_to_imgmsg(self._last_subimg, 'rgb8')
                img2pubD = self._bridge.cv2_to_imgmsg(self._last_subimgD, 'mono16')
                self._get_virtualTF()
                img2pubRGB.header.frame_id = 'RGBD_virtual'
                img2pubD.header.frame_id = 'RGBD_virtual'
                self._pubRGB.publish(img2pubRGB)         
                self._pubD.publish(img2pubD)            

def main():
    rospy.init_node('get_bestSubimage', anonymous=True)
    node = bestSubimage()
    node.run()    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
