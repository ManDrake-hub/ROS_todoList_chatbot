#!/usr/bin/python3
from tensorflow.python.ops.gen_logging_ops import Print
import rospy
import message_filters
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import pickle
import os

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id


REF_PATH = os.path.dirname(os.path.abspath(__file__))
RATE = 16000

# Load model
model = get_deep_speaker(os.path.join(REF_PATH,'deep_speaker.h5'))

n_embs = 0
X = []
y = []

TH = 0.75

def callback(data, txt):
        audio_data = np.array(data.data)

        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0

        # Processing
        ukn = get_mfcc(audio_data, RATE)

        # Prediction
        ukn = model.predict(np.expand_dims(ukn, 0))

        if len(X) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(X), 0)

            cos_dist = batch_cosine_similarity(np.array(X), emb_voice)
            
            # Matching
            id_label = dist2id(cos_dist, y, TH, mode='avg')
        
        if len(X) == 0 or id_label is None:
            rospy.Publisher('result_ID', String, 'no')
            
            #QUESTO DEVE ESSERE MANDATO AL TEXT2SPEECH PER PARLARE E POI DEVE ESSERE FATTO LO SPEECH2TEXT DELLA RISPOSTA
            c = input("Voce non conosciuta. Vuoi inserire un nuovo campione?")
            if c.lower() == 'si':

                #QUESTO DEVE ESSERE MANDATO AL TEXT2SPEECH PER PARLARE E POI DEVE ESSERE FATTO LO SPEECH2TEXT DELLA RISPOST       
                name = input("Inserisci il nome dello speaker:").lower()


                X.append(ukn[0])
                y.append(name)
            else:
                #TEXT2SPEECH ( Va bene, ciao)
                print("ciao")

        else:
            rospy.Publisher('result_ID', String, )
            #print("Ha parlato:", id_label)
        
if __name__ == '__main__':
    rospy.init_node('reidentification_node', anonymous=True)

    while not rospy.is_shutdown():
        data = message_filters.Subscriber("voice_data",Int16MultiArray,data)
        txt = message_filters.Subscriber("voice_txt", String, txt)
        ts = message_filters.TimeSynchronizer([data,txt], 10)
        ts.registerCallback(callback)