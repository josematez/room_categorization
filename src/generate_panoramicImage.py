#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class panoramicImage(object):

    def __init__(self):

        # Variables of interest
        self._publishRate = 100
        self._showImage = False

        self._bridge = CvBridge()

        self._height = 240
        self._width = 320
        self._angle = 90

        # Imagenes RGB
        self._last_image1RGB = None
        self._last_image2RGB = None 
        self._last_image3RGB = None
        self._last_image4RGB = None
        self._last_panoramicImageRGB = None

        # Imagenes Depth
        self._last_image1D = None
        self._last_image2D = None 
        self._last_image3D = None
        self._last_image4D = None
        self._last_panoramicImageD = None

        # Variables de control
        self._image1RGB_OK = False
        self._image2RGB_OK = False
        self._image3RGB_OK = False
        self._image4RGB_OK = False
        self._image1D_OK = False
        self._image2D_OK = False
        self._image3D_OK = False
        self._image4D_OK = False

        # Publishers
        self._pubRGB = rospy.Publisher('panoramicImage_RGB', Image, queue_size=10)
        self._pubD = rospy.Publisher('panoramicImage_D', Image, queue_size=10)

        # Subscribers
        rospy.Subscriber("RGBD_1_intensity", Image, self._callbackImage1RGB)
        rospy.Subscriber("RGBD_2_intensity", Image, self._callbackImage2RGB)
        rospy.Subscriber("RGBD_3_intensity", Image, self._callbackImage3RGB)
        rospy.Subscriber("RGBD_4_intensity", Image, self._callbackImage4RGB)
        rospy.Subscriber("RGBD_1_depth", Image, self._callbackImage1D)
        rospy.Subscriber("RGBD_2_depth", Image, self._callbackImage2D)
        rospy.Subscriber("RGBD_3_depth", Image, self._callbackImage3D)
        rospy.Subscriber("RGBD_4_depth", Image, self._callbackImage4D)

    # Node operation
    def run(self):
        rate = rospy.Rate(self._publishRate)

        while not rospy.is_shutdown():
            if self._image1RGB_OK and self._image2RGB_OK and self._image3RGB_OK and self._image4RGB_OK:
                # Creo la panoramica RGB
                self._last_panoramicImageRGB = np.concatenate((self._last_image3RGB, self._last_image4RGB, self._last_image1RGB, self._last_image2RGB), axis=1)
                # Convierto y publico la imagen
                panoramicImageRGB = self._bridge.cv2_to_imgmsg(self._last_panoramicImageRGB, 'rgb8')
                self._pubRGB.publish(panoramicImageRGB)

                # Reseteo las variables de control
                self._image1RGB_OK = False
                self._image2RGB_OK = False
                self._image3RGB_OK = False
                self._image4RGB_OK = False

            if self._image1D_OK and self._image2D_OK and self._image3D_OK and self._image4D_OK:
                # Creo la panoramica Depth
                self._last_panoramicImageD = np.concatenate((self._last_image3D, self._last_image4D, self._last_image1D, self._last_image2D), axis=1)
                # Convierto y publico la imagen
                panoramicImageD = self._bridge.cv2_to_imgmsg(self._last_panoramicImageD, 'mono16')
                self._pubD.publish(panoramicImageD)

                # Reseteo las variables de control
                self._image1D_OK = False
                self._image2D_OK = False
                self._image3D_OK = False
                self._image4D_OK = False
            rate.sleep()

    # Callbacks functions definition
    def _callbackImage1RGB(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image1RGB = self._bridge.imgmsg_to_cv2(data, "rgb8")
                self._last_image1RGB = self.rotate_image(self._last_image1RGB, self._angle)
                self._last_image1RGB = self._last_image1RGB[:,40:280,:]
                self._image1RGB_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage2RGB(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image2RGB = self._bridge.imgmsg_to_cv2(data, "rgb8")
                self._last_image2RGB = self.rotate_image(self._last_image2RGB, self._angle)
                self._last_image2RGB = self._last_image2RGB[:,40:280,:]
                self._image2RGB_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage3RGB(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image3RGB = self._bridge.imgmsg_to_cv2(data, "rgb8")
                self._last_image3RGB = self.rotate_image(self._last_image3RGB, self._angle)
                self._last_image3RGB = self._last_image3RGB[:,40:280,:]
                self._image3RGB_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage4RGB(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image4RGB = self._bridge.imgmsg_to_cv2(data, "rgb8")
                self._last_image4RGB = self.rotate_image(self._last_image4RGB, self._angle)
                self._last_image4RGB = self._last_image4RGB[:,40:280,:]
                self._image4RGB_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage1D(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image1D = self._bridge.imgmsg_to_cv2(data, 'passthrough')
                self._last_image1D = self.rotate_image(self._last_image1D, self._angle)
                self._last_image1D = self._last_image1D[:,40:280]
                self._image1D_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage2D(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image2D = self._bridge.imgmsg_to_cv2(data, 'passthrough')
                self._last_image2D = self.rotate_image(self._last_image2D, self._angle)
                self._last_image2D = self._last_image2D[:,40:280]
                self._image2D_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage3D(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image3D = self._bridge.imgmsg_to_cv2(data, 'passthrough')
                self._last_image3D = self.rotate_image(self._last_image3D, self._angle)
                self._last_image3D = self._last_image3D[:,40:280]
                self._image3D_OK = True
            except CvBridgeError as e:
                print(e)

    def _callbackImage4D(self, data):
        if data.height == self._height and data.width == self._width:
            try:
                self._last_image4D = self._bridge.imgmsg_to_cv2(data, 'passthrough')
                self._last_image4D = self.rotate_image(self._last_image4D, self._angle)
                self._last_image4D = self._last_image4D[:,40:280]
                self._image4D_OK = True
            except CvBridgeError as e:
                print(e)

    @staticmethod
    def rotate_image(img, angle):
        image_center = tuple(np.array(img.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(img, rot_mat, img.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result
    


def main():
    rospy.init_node('generate_panoramicImage', anonymous=True)
    node = panoramicImage()
    node.run()    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass