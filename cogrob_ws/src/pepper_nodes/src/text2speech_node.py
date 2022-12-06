#!/usr/bin/python3
from utils import Session
from pepper_nodes.srv import Text2Speech
from optparse import OptionParser
import rospy
from std_msgs.msg import String, Bool
import time
import qi

tts_event_watcher = None
'''
This class implements a ROS node able to call the Text to speech service of the robot
'''
class Text2SpeechNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port, pub):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.tts = self.session.get_service("ALTextToSpeech")
        self.tts.setLanguage("Italian")
        self.pub = pub
     
    '''
    Rececives a Text2Speech message and call the ALTextToSpeech service.
    The robot will play the text of the message
    '''
    def say(self, msg):
        print("inizio il say ")
        try:
            self.pub.publish(False)
            #MODIFICAAAAAAA METTI .DATA 
            self.tts.say(msg)
        except Exception as e:
            print(e)
            print("mi riconnetto")
            self.session.reconnect()
            self.tts = self.session.get_service("ALTextToSpeech")
            self.tts.say(msg.data)
        return "ACK"
    '''
    Starts the node and create the tts service
    '''
    def start(self):
        print("sono in start ")
        rospy.init_node("text2speech_node")
        self.say("ciao giovanni")
        #rospy.Subscriber("bot_answer", String, self.say)
        rospy.spin()



class SpeechWatcher:
    """ A class to react to the ALTextToSpeech/Status event """

    def __init__(self, app, pub):
        app.start()
        session = app.session
        self.memory = session.service("ALMemory")
        self.pub = pub
        
        self.subscriber = self.memory.subscriber("ALTextToSpeech/TextDone")
        #print(dir(type(self.memory)))
        #self.subscriber = self.memory.subscribeToEvent("ALTextToSpeech/TextDone", "text2speech_node", "on_text_done")

        self.subscriber.signal.connect(self.on_text_done)

        #self.pub = rospy.Publisher("/listen_start", String, queue_size =3)

    def on_text_done(self, value):
        print("oki")
        if value == 1:
            self.pub.publish(True)
    

if __name__ == "__main__":
    print("Avvio nodo t2s")
    time.sleep(3)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    pub = rospy.Publisher("check_listen", Bool, queue_size=10)
    print("inizio il try")
    try:
        ttsnode = Text2SpeechNode(options.ip, int(options.port), pub)
        print("text2speech created ")
        ttsnode.start()
        app = qi.Application(["SpeechWatcher", "--qi-url=" + "tcp://" + options.ip + ":" + str(options.port)])
        tts_event_watcher = SpeechWatcher(app, pub)
    except rospy.ROSInterruptException:
        pass
