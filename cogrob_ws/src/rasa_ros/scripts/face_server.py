#!/usr/bin/env python3
from rasa_ros.srv import Face, FaceResponse, Face_image
import rospy
import face_recognition
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


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
face_name = ""

def callback(msg):
    br = CvBridge()
    image = br.imgmsg_to_cv2(msg)
    
    process_this_frame = True
    global face_name
    if process_this_frame:
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(image, (0, 0), fx=0.25, fy=0.25)
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        print(len(face_locations))
        print(len(face_encodings))
        if len(face_locations)==0:
            face_name = "Unknown"
        
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            face_name = "Unknown"
            # # If a match was found in known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]
            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                face_name = known_face_names[best_match_index]
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
    cv2.imshow("Image", image)
    
    

def handle_service(req):
    global face_name
    image = image_service()
    callback(image.answer)
    
    response = FaceResponse()
    response.answer.data = face_name
    return response

    

if __name__ == '__main__':
    # Server Initialization
    rospy.init_node('face_server')
    rospy.wait_for_service('image_server')
    try:
        image_service = rospy.ServiceProxy('image_server', Face_image)
        s = rospy.Service('face_server',
                        Face, handle_service)
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    rospy.logdebug('Face server READY.')
    rospy.spin()
    