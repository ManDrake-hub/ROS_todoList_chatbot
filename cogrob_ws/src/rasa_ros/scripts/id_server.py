#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
from rasa_ros.srv import Face, FaceResponse, Face_image
from tensorflow.python.ops.gen_logging_ops import Print
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import pickle
import os
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

import requests
import warnings
import face_recognition
warnings.filterwarnings("ignore")


# Load a sample picture and learn how to recognize it.
obama_image = face_recognition.load_image_file("/home/francesca/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/face_nodes/images/obama.jpg")
obama_face_encoding = face_recognition.face_encodings(obama_image)[0]

# Load a second sample picture and learn how to recognize it.
biden_image = face_recognition.load_image_file("/home/francesca/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/face_nodes/images/biden.jpg")
biden_face_encoding = face_recognition.face_encodings(biden_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    obama_face_encoding,
    biden_face_encoding
]
known_face_names = [
    "Barack Obama",
    "Joe Biden"
]

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
face_names = []




REF_PATH = os.path.dirname(os.path.abspath(__file__))
RATE = 16000


def save_object(output_path, d):
    with open(output_path, 'wb') as f:
        pickle.dump(d, f)
    
def load_object(input_path):
    with open(input_path, 'rb') as f:
        return pickle.load(f)

# Load model
model = get_deep_speaker(os.path.join(REF_PATH,'deep_speaker.h5'))

n_embs = 0

try:
    X = load_object("audio.pkl")
    Y = load_object("name.pkl")
except:
    X = []
    Y = []

TH = 0.75

class Nodo(object):
    def __init__(self):
        # Params
        self.br = CvBridge()
    
    '''
    This method receives a Image message and converts it to numpy array, then show the image opening a window
    '''
    def callback(self, msg):
        global name
        image = self.br.imgmsg_to_cv2(msg)
        process_this_frame = True
        if process_this_frame:
            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(image, (0, 0), fx=0.25, fy=0.25)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            name = ""
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # # If a match was found in known_face_encodings, just use the first one.
                # if True in matches:
                #     first_match_index = matches.index(True)
                #     name = known_face_names[first_match_index]

                # Or instead, use the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]

                face_names.append(name)

        process_this_frame = not process_this_frame

        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        # Display the resulting image
        #cv2.imshow('Video', image)
        #cv2.destroyAllWindows()
        #cv2.waitKey(50)
    
    '''
    THis method subscribes the node to specific topic and starts the node loop
    '''
    def start(self):
        global name
        # Subscriber
        image = image_service()
        self.callback(image.answer)
        print(name)
        return name

def handle_service(req):
    global face_name
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
    if input_text.data == "":
        print(f"lenX{len(X)}")
        if len(X) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(X), 0)
            cos_dist = batch_cosine_similarity(np.array(X), emb_voice)
            # Matching
            id_label = dist2id(cos_dist, Y, TH, mode='avg')
        if id_label is None:
            print("non ti riconosco")
            #TO DO: Face server
            my_node = Nodo()
            face_name = my_node.start()
            print("output faccia")
            print(face_name)
            if face_name == "Unknown" or face_name == "":
                print("non ti so nemmeno io")
                response.answer.data = ""
            else: 
                print("ti so")
                response.answer.data = face_name
        else:
            response.answer.data = id_label
    else: 
        X.append(ukn[0])
        Y.append(input_text.data)
        save_object("audio.pkl", X)
        save_object("name.pkl", Y)
    return response




if __name__ == '__main__':
    # Server Initialization
    rospy.init_node('id_server')

    rospy.wait_for_service('image_server')
    try:
        image_service = rospy.ServiceProxy('image_server', Face_image)
        #face_service = rospy.ServiceProxy('face_server', Face)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    s = rospy.Service('id_server',
                        ID, handle_service)
    rospy.logdebug('ID server READY.')
    rospy.spin()
