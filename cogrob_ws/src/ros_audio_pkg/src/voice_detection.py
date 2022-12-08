#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np
import qi
import time
from optparse import OptionParser
import speech_recognition as sr

class SpeechWatcher:
    """ A class to react to the ALTextToSpeech/Status event """

    def __init__(self, app):
        app.start()
        session = app.session
        self.memory = session.service("ALMemory")
        
        self.subscriber_start = self.memory.subscriber("ALTextToSpeech/TextStarted")
        self.subscriber_done = self.memory.subscriber("ALTextToSpeech/TextDone")
        #print(dir(type(self.memory)))
        #self.subscriber = self.memory.subscribeToEvent("ALTextToSpeech/TextDone", "text2speech_node", "on_text_done")

        self.subscriber_start.signal.connect(self.on_text_start)
        self.subscriber_done.signal.connect(self.on_text_done)

        #self.pub = rospy.Publisher("/listen_start", String, queue_size =3)

    def on_text_start(self, value):
        self.value_s = value
        if value == 1:
            print("start")

    def on_text_done(self, value):
        self.value_d = value
        if value == 1:
            print("done")

# this is called from the background thread
def callback(recognizer, audio):
    if not tts_event_watcher.value_s or tts_event_watcher.value_d:
        print("ascolto")
        data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
        data_to_send = Int16MultiArray()
        data_to_send.data = data
        pub.publish(data_to_send)

if __name__ == "__main__":
    tts_event_watcher = None
    rospy.init_node('voice_detection_node', anonymous=True)
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    app = qi.Application(["SpeechWatcher", "--qi-url=" + "tcp://" + options.ip + ":" + str(options.port)])
    tts_event_watcher = SpeechWatcher(app)
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
    stop_listening = r.listen_in_background(m, callback)
    pub = rospy.Publisher('mic_data', Int16MultiArray, queue_size=10)
    rospy.spin()