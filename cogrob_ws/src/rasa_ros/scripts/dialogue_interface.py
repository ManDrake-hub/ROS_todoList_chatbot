#!/usr/bin/env python3

import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue, ID
from pepper_nodes.srv import Text2Speech
from rasa_ros.msg import SAT
from std_msgs.msg import String, Int16MultiArray
import message_filters

class TerminalInterface:
    '''
    Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal
    '''
    def __init__(self) -> None:
        self.changed = False
        self.AIN = False
        self.Name = String()
        self.Name.data = None

    def callback(self, data):
        print("parte callback")
        self.txt = data.text
        self.data = data.audio
        if self.AIN == False:
            if self.Name.data == None:
                try:
                    print("parte il try")
                    vuoto = String()
                    vuoto.data = " "
                    id_answer = id_service(self.data, vuoto)
                    print("risposta: %s"%id_answer)
                    if id_answer.answer.data == '':
                        print("Come ti chiami?")
                        pub.publish("Come ti chiami?")
                        self.AIN = True
                    else:
                        print("Nome salvato")
                        self.Name.data = id_answer.answer.data
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
            else:
                try:
                    id_answer = id_service(self.data, self.Name)
                    print("salvo")
                    bot_answer = dialogue_service(self.txt)
                    pub.publish(bot_answer.answer)
                    print("bot answer: %s"%bot_answer.answer)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
            if self.txt == "Arrivederci":
                print("ho chiuso")
                self.Name.data = None
        else:
            print("no distanza")
            self.AIN = False
            self.Name.data = self.txt
            id_service(self.data, self.Name)
        

    #    self.changed = True
#
    #def has_changed(self):
    #    return self.changed
#
    #def get_text(self):
    #    self.changed = False
    #    return self.message

if __name__ == '__main__':
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    rospy.wait_for_service('id_server')
    print("IN:")
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    id_service = rospy.ServiceProxy('id_server', ID)
    terminal = TerminalInterface()
    pub = rospy.Publisher("bot_answer", String, queue_size=10)
    rospy.Subscriber("voice_txt_data", SAT, terminal.callback)
    rospy.spin()
    #txt_sub = message_filters.Subscriber("/voice_txt", String)
    #data_sub = message_filters.Subscriber("voice_data", Int16MultiArray)
    #ts = message_filters.TimeSynchronizer([data_sub, txt_sub], 10)
    #ts.registerCallback(terminal.callback)

    #gerry = False
    #while not rospy.is_shutdown():
    #    if gerry == False:
    #        print("IN:")
    #        gerry = True
    #    if terminal.has_changed():
    #        message = terminal.get_text()
    #        if message == 'exit': 
    #            break
    #        try:
    #            print("bot answer: %s"%bot_answer.answer)
    #        except rospy.ServiceException as e:
    #            print("Service call failed: %s"%e)