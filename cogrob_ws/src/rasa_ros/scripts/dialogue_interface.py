#!/usr/bin/env python3

import rospy
import warnings
warnings.filterwarnings("ignore")
from rasa_ros.srv import Dialogue
from pepper_nodes.srv import Text2Speech
from std_msgs.msg import String
import numpy as np
from tensorflow.python.ops.gen_logging_ops import Print
import message_filters
from std_msgs.msg import Int16MultiArray
import pickle
import os
from ros_audio_pkg.src.identification.deep_speaker.audio import get_mfcc
from ros_audio_pkg.src.identification.deep_speaker.model import get_deep_speaker
from ros_audio_pkg.src.identification.utils import batch_cosine_similarity, dist2id

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

class ID_Recognition:
    
    def __init__(self) -> None:
        self.REF_PATH = os.path.dirname(os.path.abspath(__file__))
        self.RATE = 16000
        # Load model
        self.model = get_deep_speaker(os.path.join(self.REF_PATH,'deep_speaker.h5'))
        self.n_embs = 0
        self.X = []
        self.y = []
        self.TH = 0.75

    def callback(self, data, txt):
        audio_data = np.array(data.data)
        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0
        # Processing
        ukn = get_mfcc(audio_data, self.RATE)
        # Prediction
        ukn = self.model.predict(np.expand_dims(ukn, 0))
        if len(self.X) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(X), 0)
            cos_dist = batch_cosine_similarity(np.array(X), emb_voice)
            # Matching
            id_label = dist2id(cos_dist, self.y, self.TH, mode='avg')
            self.done = True
        if id_label is not None:
            self.X.append(ukn[0])
            self.y.append(id_label)
            self.bot_answer = txt
        else:
            # ------ TO DO using self.done------
            self.bot_answer = "Voce non conosciuta. Come ti chiami?"

    def has_done(self):
        return self.done

    def get_bot_answer(self):
        return self.bot_answer

def main():
    terminal = TerminalInterface()
    id = ID_Recognition()
    pub = rospy.Publisher("bot_answer", String, queue_size=10)
    data = message_filters.Subscriber("voice_data",Int16MultiArray,data)
    txt = message_filters.Subscriber("voice_txt", String, txt)
    ts = message_filters.TimeSynchronizer([data,txt], 10)
    ts.registerCallback(id.callback)
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
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
                if id.get_bot_answer == txt:
                    bot_answer = dialogue_service(txt)
                    pub.publish(bot_answer)
                    print("bot answer: %s"%bot_answer.answer)
                else:
                    pub.publish(id.get_bot_answer)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass
