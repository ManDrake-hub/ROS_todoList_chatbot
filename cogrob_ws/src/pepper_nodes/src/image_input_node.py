#!/usr/bin/python3
from rasa_ros.srv import Face_image,Face_imageResponse
from cv_bridge import CvBridge
from utils import Session
from optparse import OptionParser
from sensor_msgs.msg import Image
import numpy as np
import rospy
import cv2


TOP_CAMERA = 0
BOTTOM_CAMERA = 1
DEPTH_CAMERA = 2

RES_120P = 0
RES_240P = 1
RES_480P = 2
RES_960P = 3

COLORSPACE_GRAYSCALE = 8
COLORSPACE_RGB = 13

MODE_RGB = 0
MODE_DEPTH = 1
MODE_RGBD = 2

class LocalNode:
    """
    This class configures a ROS node able to read the video stream from the local camera
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.fps = 30
    
    def handle_service(self,req):
        msg = Image()
        rate = rospy.Rate(self.fps)
        cap = cv2.VideoCapture(0)
        print("ho aperto e preso l'immagine")
        # Capture frame-by-frame
        ret, frame = cap.read()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame)
            msg.header.stamp = rospy.Time.now()
        rate.sleep()
        return msg

'''
This class configures a ROS node able to read the video stream from the robot's camera
'''
class ImageInputNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services. Then opens a video stream with the Pepper camera.
    The video stream is then published on a specific topic. Camera parameters can be configured using the above variables
    '''
    def __init__(self, ip, port, resolution=RES_480P, rgb_camera=TOP_CAMERA, fps=30):
        self.session = Session(ip, port)
        self.fps = fps

        if resolution == RES_120P:
            self.width, self.height = 160, 120
        elif resolution == RES_240P:
            self.width, self.height = 320, 240
        elif resolution == RES_480P:
            self.width, self.height = 640, 480
        elif resolution == RES_960P:
            self.width, self.height = 1280, 960
        else:
            self.width, self.height = None, None
        self.camera = self.session.get_service("ALVideoDevice")
        self.rgb_sub = self.camera.subscribeCamera("RGB Stream", rgb_camera, resolution, COLORSPACE_RGB, self.fps)
        if not self.rgb_sub:
            raise Exception("Camera is not initialized properly")
        self.bridge = CvBridge()
    
    '''
    Retrieves the latest image from the video source. Then tranforms it into a numpy array
    @return: Returns the image frame like a numpy array
    '''
    def get_color_frame(self):
        raw_rgb = self.camera.getImageRemote(self.rgb_sub)
        image = np.frombuffer(raw_rgb[6], np.uint8).reshape(raw_rgb[1], raw_rgb[0], 3)
        return image
    
    '''
    Returns the horizontal and vertical field of view of the camera, expressed in radians.
    '''
    def get_fov(self, mode="RGB"):
        hfov, vfov = 0, 0
        if mode == "RGB":
            hfov = 57.2 * np.pi / 180
            vfov = 44.3 * np.pi / 180
        return hfov, vfov
    
    '''
    Unregisters a module from ALVideoDevice
    '''
    def stop(self):
        self.camera.unsubscribe(self.rgb_sub)
    
    '''
    Starts the node and opens the video stream
    '''
    def handle_service(self, req):
        rate = rospy.Rate(self.fps)
        frame = self.get_color_frame()
        print("ho aperto e preso l'immagine")
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame)
            msg.header.stamp = rospy.Time.now()
        rate.sleep()
        return msg

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    try:
        rospy.init_node("image_input_node")
        #TEST WEBCAM
        image_input = LocalNode()
        s = rospy.Service('image_server',
                        Face_image, image_input.handle_service)
        rospy.logdebug('Face server READY.')
        rospy.spin()

        #NOT WORK ON 230 BECAUSE THE WEBCAM IS BLURRED
        #image_input = ImageInputNode(options.ip, int(options.port))
        #s = rospy.Service('image_server',
        #                Face_image, image_input.handle_service)
        #rospy.logdebug('Face server READY.')
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass

    
