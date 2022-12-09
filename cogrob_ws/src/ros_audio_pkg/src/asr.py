#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
from ros_audio_pkg.msg import SAT

from speech_recognition import AudioData
import speech_recognition as sr

# Initialize a Recognizer
r = sr.Recognizer()

# Init node
rospy.init_node('speech_recognition', anonymous=True)
pub = rospy.Publisher('voice_txt_data', SAT, queue_size=10)


# this is called from the background thread
def callback(audio):
    data = np.array(audio.data,dtype=np.int16)
    audio_data = AudioData(data.tobytes(), 16000, 2)

    try:
        spoken_text= r.recognize_google(audio_data, language='it-IT')
        print("Google Speech Recognition pensa tu abbia detto: " + spoken_text)
        msg = SAT()
        msg.audio = audio
        msg.text = spoken_text
        pub.publish(msg)
    except sr.UnknownValueError:
        print("Google Speech Recognition non riesce a capire da questo file audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def listener():
    rospy.Subscriber("mic_data", Int16MultiArray, callback)

    rospy.spin()



if __name__ == '__main__':
    listener()
#    rospy.wait_for_service('speech_ID')
#    try: 
#        speech_ID = rospy.ServiceProxy('speech_ID', Speech_ID)
#        resp = Speech_ID(audio)
#        
#    except rospy.ServiceException as e:
#        print("Service call failed: %s"%e)