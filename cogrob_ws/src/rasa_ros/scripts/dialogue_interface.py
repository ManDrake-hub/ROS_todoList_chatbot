#!/usr/bin/env python3

import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import String


class TerminalInterface:
    '''
    Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal
    '''
    def __init__(self) -> None:
        self.has_changed = False
        
    def callback(self, data):
        self.message = data
        self.has_changed = True

    def has_changed(self):
        return self.has_changed

    def get_text(self):
        self.has_changed = False
        return self.message

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    terminal = TerminalInterface()
    rospy.Subscriber("voice_txt", String, terminal.callback)

    while not rospy.is_shutdown():
        if terminal.has_changed():
            message = terminal.get_text()
            if message == 'exit': 
                break
            try:
                bot_answer = dialogue_service(message)
                terminal.set_text(bot_answer.answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass