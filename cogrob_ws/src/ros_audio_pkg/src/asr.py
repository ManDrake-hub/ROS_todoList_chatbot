#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
from ros_audio_pkg.msg import SAT
#import whisper
import soundfile as sf

from speech_recognition import AudioData
import speech_recognition as sr


class Transcriber:
    def __init__(self, r: sr.Recognizer, input_topic="mic_data", output_topic="voice_txt_data") -> None:
        """
        Initializes a transcriber that will subscrive to the input_topic and publish
        the output text to the output_topic.
        """
        self.r = r
        # Create publisher for the transcription
        self.pub = rospy.Publisher(output_topic, SAT, queue_size=10)
        # Subscribe to the topic mic_data to transcribe incoming microphone data
        rospy.Subscriber(input_topic, Int16MultiArray, self.callback)

    # This is called from the background thread
    def callback(self, audio):
        # Read the audio's data
        data = np.array(audio.data, dtype=np.int16)
        audio_data = AudioData(data.tobytes(), 16000, 2)

        try:
            # Try to get transcription
            spoken_text= self.r.recognize_google(audio_data, language='it-IT')
            print("Google Speech Recognition pensa tu abbia detto: " + spoken_text)
            msg = SAT()
            msg.audio = audio
            msg.text.data = spoken_text
            self.pub.publish(msg)
        except sr.UnknownValueError:
            print("Google Speech Recognition non riesce a capire da questo file audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))


if __name__ == '__main__':
    # Init node
    rospy.init_node('speech_recognition', anonymous=True)
    # Initialize a Recognizer
    r = sr.Recognizer()
    # Start transcription
    t = Transcriber(r, input_topic="mic_data", output_topic="voice_txt_data")
    rospy.spin()