#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np

import time
import speech_recognition as sr


class Microphone:
    def __init__(self, r: sr.Recognizer, m: sr.Microphone, output_topic="mic_data", disable_topic="pepper_say"):
        """
        Initializes the microphone and starts listening on a background thread.
        The audio data will be published on the output_topic.

        The background thread can be stopped/started by sending a boolean to the topic disable_topic.
        """
        self.r = r
        self.m = m
        self.pub = rospy.Publisher(output_topic, Int16MultiArray, queue_size=10)
        self.stop_listening = r.listen_in_background(m, self.callback_audio_publish)
        rospy.Subscriber(disable_topic, Bool, self.callback_audio_disable)

    def callback_audio_publish(self, recognizer, audio):
        """Get the raw data from the audio stream and publishes it on "mic_data" """
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        self.pub.publish(data_to_send)

    def callback_audio_disable(self, value):
        """Get the bool from the topic and if True, stop listening in the background"""
        if not value.data:
            self.stop_listening = self.r.listen_in_background(self.m, self.callback)
        else:
            self.stop_listening()


if __name__ == "__main__":
    rospy.init_node('voice_detection_node', anonymous=True)

    # Initialize a Recognizer
    r = sr.Recognizer()

    # Init Audio source
    m = sr.Microphone(device_index=None,
                        sample_rate=16000,
                        chunk_size=1024)

    # Calibration levels to adjust for the environment
    print("Calibrating...")
    with m as source:
        r.adjust_for_ambient_noise(source, duration=3)
    print("Calibration finished")

    # Start listening in the background
    print("Recording...")
    mic = Microphone(r, m, output_topic="mic_data", disable_topic="pepper_say")
    rospy.spin()