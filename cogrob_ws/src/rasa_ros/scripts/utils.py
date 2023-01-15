#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
from rasa_ros.srv import Face_image
from tensorflow.python.ops.gen_logging_ops import Print
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import pickle
import os
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import pathlib

from identification.deep_speaker.audio import get_mfcc
from identification.deep_speaker.model import get_deep_speaker
from identification.utils import batch_cosine_similarity, dist2id

import requests
import warnings
import face_recognition
warnings.filterwarnings("ignore")

def save_object(output_path, d):
    with open(output_path, 'wb') as f:
        pickle.dump(d, f)
    
def load_object(input_path):
    with open(input_path, 'rb') as f:
        return pickle.load(f)

class AudioRecognizer:
    def __init__(self, threshold: float=0.75, audio_rate: int=16000) -> None:
        # Load dataset
        self.folder = pathlib.Path(__file__).parent.parent.resolve()
        self.x = load_object(self.folder+"/audio.pkl") if os.path.exists(self.folder+"/audio.pkl") else []
        self.y = load_object(self.folder+"/name.pkl") if os.path.exists(self.folder+"/name.pkl") else []
        # Load the audio model
        self.model = get_deep_speaker(os.path.join(os.path.dirname(os.path.abspath(__file__)), './deep_speaker.h5'))
        self.audio_rate = audio_rate
        # set recognition threshold
        self.threshold = threshold

    def add_sample(self, x, y) -> None:
        self.x.append(x)
        save_object(self.folder+"/audio.pkl", self.x)
        self.y.append(y)
        save_object(self.folder+"/name.pkl", self.y)

    def get_embeddings(self, audio):
        """Get audio embeddings"""
        audio_data = np.array(audio)
        # to float32
        audio_data = audio_data.astype(np.float32, order='C') / 32768.0
        # Processing
        ukn = get_mfcc(audio_data, self.audio_rate)
        # Predict audio embeddings
        return self.model.predict(np.expand_dims(ukn, 0))

    def recognize(self, audio) -> str:
        ukn = self.get_embeddings(audio)

        if len(self.x) > 0:
            # Distance between the sample and the support set
            emb_voice = np.repeat(ukn, len(self.x), 0)
            cos_dist = batch_cosine_similarity(np.array(self.x), emb_voice)
            # Matching
            name = dist2id(cos_dist, self.y, self.threshold, mode='avg')
            if name is None:
                name = ""
            return name
        return ""

class FaceRecognizer:
    def __init__(self) -> None:
        self.br = CvBridge()
        # Load dataset
        self.folder = pathlib.Path(__file__).parent.parent.resolve()
        self.x = load_object(self.folder+"/face.pkl") if os.path.exists(self.folder+"/face.pkl") else []
        self.y = load_object(self.folder+"/face_name.pkl") if os.path.exists(self.folder+"/face_name.pkl") else []
        # Connect to image service proxy
        self.image_service = rospy.ServiceProxy('image_server', Face_image)

    def add_sample(self, x, y) -> None:
        self.x.append(x)
        save_object(self.folder+"/face.pkl", self.x)
        self.y.append(y)
        save_object(self.folder+"/face_name.pkl", self.y)

    def get_image(self):
        """Capture an image from image service and convert it to cv2"""
        msg = self.image_service().answer
        return self.br.imgmsg_to_cv2(msg)

    def get_embeddings(self, image):
        """Get face embeddings from cv2 image"""
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(image, (0, 0), fx=1, fy=1)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        return face_recognition.face_encodings(rgb_small_frame, face_locations)

    def recognize(self) -> str:
        if len(self.x) == 0:
            return ""

        # Capture an image
        image = self.get_image()
        # Get list of face embeddings in image
        face_encoding = self.get_embeddings(image)
        # If no face embeddings have been found, return ""
        if len(face_encoding) == 0:
            return ""
        # Get the first face embedding found
        face_encoding = face_encoding[0]
        
        # See if the face is a match for the known face(s)
        matches = face_recognition.compare_faces(self.x, face_encoding)

        # Use the known face with the smallest distance to the new face
        face_distances = face_recognition.face_distance(self.x, face_encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = self.y[best_match_index]
        if name is None:
            return ""
        return name