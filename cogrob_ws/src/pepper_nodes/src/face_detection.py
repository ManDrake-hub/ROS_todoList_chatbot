#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Text2Speech
from optparse import OptionParser
import rospy
from std_msgs.msg import String, Bool
import time
import qi

'''
This class implements a ROS node able to call the Text to speech service of the robot
'''
class FaceNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        #self.tts = self.session.get_service("ALFaceDetection")
        self.motion = self.session.get_service("ALMotion")
        self.tracker = self.session.get_service("ALTracker")
        #self.tracker.faceinfo
     
    '''
    Rececives a Text2Speech message and call the ALTextToSpeech service.
    The robot will play the text of the message
    '''
    def learnFace(self, msg):
        try:
            self.tts.learnFace(msg)
        except Exception as e:
            print(e)
            self.session.reconnect()
            self.tts = self.session.get_service("ALFaceDetection")
            self.tts.learnFace(msg)
        return "ACK"
    '''
    Starts the node and create the tts service
    '''
    def start(self):
        rospy.init_node("facenode")
        #self.learnFace("Luigi")
            
    def track(self, faceSize):

        # Add target to track.
        targetName = "Face"
        faceWidth = faceSize
        
        self.tracker.registerTarget(targetName, faceWidth)

        # Then, start tracker.
        self.tracker.track(targetName)

        print ("ALTracker successfully started, now show your face to robot!")
        print ("Use Ctrl+c to stop this script.")

        try:
            while True:
                print(self.tracker.getTargetPosition(0))
                time.sleep(1)
        except KeyboardInterrupt:
            print ("Interrupted by user")
            print ("Stopping...")

        # Stop tracker.
        self.tracker.stopTracker()
        self.tracker.unregisterAllTargets()

        print ("ALTracker stopped.")


if __name__ == "__main__":
    time.sleep(3)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    try:
        ttsnode = FaceNode(options.ip, int(options.port))
        ttsnode.start()
        ttsnode.track(30)
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(e)

