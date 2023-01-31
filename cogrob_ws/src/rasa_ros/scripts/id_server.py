#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
import rospy
import warnings
warnings.filterwarnings("ignore")
from utils import AudioRecognizer, FaceRecognizer
import cv2


def handle_wrapper(audio_recognizer: AudioRecognizer, face_recognizer: FaceRecognizer):
    """
    Wrapper around the service handler to provide audio and face recognizers.
    """
    def handle_service(req):
        audio = req.audio
        input_text = req.input_text
        response = IDResponse()
        response.answer.data = ""

        if input_text.data == "":
            # If the name is empty then it means that a recognition has been requested
            print("Using audio recognition")
            # Try to get label from audio recognizer
            id_label = audio_recognizer.recognize(audio.data)
            if id_label == "":
                # If no label has been found, fallback on the face recognizer
                print("Using face recognizion fallback")
                id_label = face_recognizer.recognize()
            print(f"Persona riconosciuta: {id_label}")
            # Set the response to the label, be it empty of the name of the recognized user
            response.answer.data = id_label
        else:
            # Else (name has been provided), save the new relationships
            audio_recognizer.add_sample(audio_recognizer.get_embeddings(audio.data)[0], input_text.data)
            print("Added new audio sample")
            # Capture an image from the camera
            image = face_recognizer.get_image()
            # Get face embeddings
            face_embeddings = face_recognizer.get_embeddings(image)
            if len(face_embeddings) > 0:
                # If face embeddings have been found, add relationship
                face_recognizer.add_sample(face_embeddings[0], input_text.data)
                print("Added new face sample")
        return response
    return handle_service


if __name__ == '__main__':
    # Server Initialization
    rospy.init_node('id_server')
    rospy.wait_for_service('image_server')
    
    audio_recognizer = AudioRecognizer()
    face_recognizer = FaceRecognizer()

    print("init successful")

    rospy.Service('id_server', ID, handle_wrapper(audio_recognizer, face_recognizer))
    rospy.spin()
