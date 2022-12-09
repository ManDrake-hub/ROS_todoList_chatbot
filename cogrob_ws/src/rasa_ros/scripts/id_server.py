#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
from tensorflow.python.ops.gen_logging_ops import Print
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import pickle
import os

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

import requests
import warnings
warnings.filterwarnings("ignore")

REF_PATH = os.path.dirname(os.path.abspath(__file__))
RATE = 16000

# Load model
model = get_deep_speaker(os.path.join(REF_PATH,'deep_speaker.h5'))

n_embs = 0
X = []
Y = []

TH = 0.75

def handle_service(req):
    audio = req.audio
    id_label = None
    input_text = req.input_text
    response = IDResponse()
    audio_data = np.array(audio.data)
    # to float32
    audio_data = audio_data.astype(np.float32, order='C') / 32768.0
    # Processing
    ukn = get_mfcc(audio_data, RATE)
    # Prediction
    ukn = model.predict(np.expand_dims(ukn, 0))
    print("input_text: %s"%input_text)   
    if input_text.data == " ":
        if len(X) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(X), 0)
            cos_dist = batch_cosine_similarity(np.array(X), emb_voice)

            # Matching
            id_label = dist2id(cos_dist, Y, TH, mode='avg')
        print("id_label: %s"%id_label)
        if id_label is None:
            response.answer.data = ""
        else:
            response.answer.data = id_label
    else: 
        print("salvo veramente")
        print("sono Y")
        for i in range(len(Y)):
            print(Y[i])
        X.append(ukn[0])
        Y.append(input_text.data)
    return response

if __name__ == '__main__':
    # Server Initialization
    rospy.init_node('id_server')
    s = rospy.Service('id_server',
                        ID, handle_service)
    rospy.logdebug('ID server READY.')
    rospy.spin()