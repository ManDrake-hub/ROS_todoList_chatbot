#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
import rospy
import warnings
warnings.filterwarnings("ignore")
from utils import AudioRecognizer, FaceRecognizer

def handle_wrapper(audio_recognizer: AudioRecognizer, face_recognizer: FaceRecognizer):
    def handle_service(req):
        audio = req.audio
        input_text = req.input_text
        response = IDResponse()
        
        if input_text.data == "":
            id_label = audio_recognizer.recognize(audio.data)

            if id_label == "":
                id_label = face_recognizer.recognize()
            response.answer.data = id_label
        else:
            audio_recognizer.add_sample(audio_recognizer.get_embeddings(audio.data)[0], input_text.data)
            face_embeddings = face_recognizer.get_embeddings(face_recognizer.get_image())
            if len(face_embeddings) > 0:
                face_recognizer.add_sample(face_recognizer.get_embeddings(face_recognizer.get_image())[0], input_text.data)
        return response
    return handle_service

if __name__ == '__main__':
    # Server Initialization
    rospy.init_node('id_server')
    rospy.wait_for_service('image_server')
    
    audio_recognizer = AudioRecognizer()
    face_recognizer = FaceRecognizer()

    rospy.Service('id_server', ID, handle_wrapper(audio_recognizer, face_recognizer))
    rospy.spin()
