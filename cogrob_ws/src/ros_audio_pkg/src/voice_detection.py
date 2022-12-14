#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np

import time
import speech_recognition as sr


class Microphone:
    def __init__(self):
        self.stop_listening = r.listen_in_background(m, self.callback)
    # this is called from the background thread
    def callback(self, recognizer, audio):
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        pub.publish(data_to_send)

    def callback_2(self, value):
        if value == False:
            self.stop_listening = r.listen_in_background(m, self.callback)
        else:
            self.stop_listening()

if __name__ == "__main__":
    pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
    rospy.init_node('voice_detection_node', anonymous=True)
    # Initialize a Recognizer
    r = sr.Recognizer()

    #import sounddevice as sd
    #sd.query_devices()

    # Audio source
    m = sr.Microphone(device_index=None,
                        sample_rate=16000,
                        chunk_size=1024)

    # Calibration within the environment
    # we only need to calibrate once, before we start listening
    print("Calibrating...")
    with m as source:
        r.adjust_for_ambient_noise(source,duration=3)  
    print("Calibration finished")

    # start listening in the background
    # `stop_listening` is now a function that, when called, stops background listening
    print("Recording...")
    mic = Microphone()
    rospy.Subscriber('pepper_say', Bool, mic.callback_2)


rospy.spin()