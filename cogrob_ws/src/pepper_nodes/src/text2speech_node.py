#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Text2Speech
from optparse import OptionParser
import rospy
from std_msgs.msg import String, Bool
import time
import qi
<<<<<<< HEAD
=======

>>>>>>> Text2Speech
'''
This class implements a ROS node able to call the Text to speech service of the robot
'''
class Text2SpeechNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.tts = self.session.get_service("ALTextToSpeech")
<<<<<<< HEAD
        self.tts.setLanguage("Italian")
     
=======
        #self.tts = self.session.get_service("ALAnimatedSpeech")
        self.tts.setLanguage("Italian")
        self.tts.setVolume(0.5)
    
>>>>>>> Text2Speech
    '''
    Rececives a Text2Speech message and call the ALTextToSpeech service.
    The robot will play the text of the message
    '''
    def say(self, msg):
<<<<<<< HEAD
        print("inizio il say ")
        try:
            self.tts.say(msg.data)
        except Exception as e:
            print(e)
            print("mi riconnetto")
            self.session.reconnect()
            self.tts = self.session.get_service("ALTextToSpeech")
            self.tts.say(msg.data)
=======
        pub.publish(True)
        try:
            self.tts.say(msg.data)
            #configuration = {"bodyLanguageMode":"contextual"}
            #self.tts.say(msg.data, configuration)
            pub.publish(False)
        except Exception as e:
            print(e)
            self.session.reconnect()
            self.tts = self.session.get_service("ALTextToSpeech")
            #self.tts = self.session.get_service("ALAnimatedSpeech")
            #configuration = {"bodyLanguageMode":"contextual"}
            self.tts.say(msg.data)
            #self.tts.say(msg.data, configuration)
            pub.publish(False)
>>>>>>> Text2Speech
        return "ACK"
    '''
    Starts the node and create the tts service
    '''
    def start(self):
        print("sono in start ")
        rospy.init_node("text2speech_node")
        rospy.Subscriber("bot_answer", String, self.say)
        rospy.spin()
    

if __name__ == "__main__":
    print("Avvio nodo t2s")
    time.sleep(3)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
<<<<<<< HEAD
    print("inizio il try")
=======
    pub = rospy.Publisher('pepper_say', Bool, queue_size=10)
>>>>>>> Text2Speech
    try:
        ttsnode = Text2SpeechNode(options.ip, int(options.port))
        print("text2speech created ")
        ttsnode.start()
    except rospy.ROSInterruptException:
        pass
