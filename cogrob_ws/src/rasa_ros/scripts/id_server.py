#!/usr/bin/env python3
from rasa_ros.srv import ID, IDResponse
import rospy
import warnings
warnings.filterwarnings("ignore")
from utils import AudioRecognizer, FaceRecognizer
import cv2

def handle_wrapper(audio_recognizer: AudioRecognizer, face_recognizer: FaceRecognizer):
    def handle_service(req):
        print("req received")
        audio = req.audio
        input_text = req.input_text
        response = IDResponse()
        response.answer.data = ""

        if input_text.data == "":
            print("recognize with audio")
            id_label = audio_recognizer.recognize(audio.data)
            print("recognize with audio done")

            if id_label == "":
                print("recognize with face")
                id_label = face_recognizer.recognize()
                print("recognize with face done")
            print(f'id lable {id_label}')
            response.answer.data = id_label
        else:
            print("sto aggiungendo audio embeddings")
            audio_recognizer.add_sample(audio_recognizer.get_embeddings(audio.data)[0], input_text.data)
            print("sto aggiungendo face embeddings")
            image = face_recognizer.get_image()
            print(image.shape)
            # cv2.imshow("", image)
            # cv2.waitKey(0)
            face_embeddings = face_recognizer.get_embeddings(image)
            print(face_embeddings)
            if len(face_embeddings) > 0:
                print("aggiungo merdeeeeee")
                face_recognizer.add_sample(face_embeddings[0], input_text.data)
        return response
    print("handler has been given")
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
