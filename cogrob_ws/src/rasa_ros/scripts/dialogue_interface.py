#!/usr/bin/env python3

import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue
from pepper_nodes.srv import Text2Speech
from std_msgs.msg import String


class TerminalInterface:
    '''
    Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal
    '''
    def __init__(self) -> None:
        self.changed = False
        
    def callback(self, data):
        self.message = data.data
        self.changed = True

    def has_changed(self):
        return self.changed

    def get_text(self):
        self.changed = False
        return self.message

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    terminal = TerminalInterface()
    pub = rospy.Publisher("bot_answer", String, queue_size=10)
    rospy.Subscriber("voice_txt", String, terminal.callback)
    gerry = False
    while not rospy.is_shutdown():
        if gerry == False:
            print("IN:")
            gerry = True
        if terminal.has_changed():
            message = terminal.get_text()
            if message == 'exit': 
                break
            try:
                bot_answer = dialogue_service(message)
                #terminal.set_text(bot_answer.answer)
                pub.publish(bot_answer)
                print("bot answer: %s"%bot_answer.answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
