#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Text2Speech
from optparse import OptionParser
import rospy
from std_msgs.msg import String, Bool
import time
import qi

class AlertNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.audio_proxy = self.session.get_service("ALAudioPlayer")
    
    
    def alert(self):
        try:
            print("try")
            import os
            print(os.path.isfile("/home/francesca/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/ringtone_1_46486.wav"))
            print(self.audio_proxy.getInstalledSoundSetsList())
            self.audio_proxy.playWebStream("https://www.youtube.com/watch?v=WfhLLBKdD5w&ab_channel=Melamarcia5535",0.2,0)
            #fileID = self.audio_proxy.loadFile("/home/francesca/Scrivania/1.wav")
            #self.audio_proxy.play(fileID)
        except Exception as e:
            print(e)
            self.session.reconnect()
            self.audio_proxy = self.session.get_service("ALAudioPlayer")
            self.audio_proxy.playWebStream("https://www.youtube.com/watch?v=WfhLLBKdD5w&ab_channel=Melamarcia5535",0.2,0)
        return "ACK"
    
    '''
    Starts the node and creates the services
    '''
    def start(self):
        rospy.init_node("audio_node")

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.230")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    try:
        node_audio = AlertNode(options.ip, int(options.port))
        node_audio.start()
        node_audio.alert()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
        pass
