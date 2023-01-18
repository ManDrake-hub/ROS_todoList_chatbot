#!/usr/bin/python3
import os
from std_msgs.msg import Int16MultiArray, Bool
import datetime
import rospy
from std_msgs.msg import String
from optparse import OptionParser
from utils import Session
from apscheduler.schedulers.background import BackgroundScheduler
from utils import check_alerts
import pathlib


class AlertNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.audio_proxy = self.session.get_service("ALAudioPlayer")
        self.talking = False
        self.last_alerts = []
    
    def alert(self):
        """
        Plays a sound if any alarm is pending without interrupting the conversation.
        """
        folder = str(pathlib.Path(__file__).parent.parent.parent.parent.parent.resolve()) + "/chatbot"

        alerts = check_alerts(folder_json=folder, alert_length=15)
        if len(alerts) == 0 or self.last_alerts == alerts or self.talking:
            return
        self.last_alerts = alerts
        try:
            fileID = self.audio_proxy.loadFile("/home/nao/alert.mp3")
            self.audio_proxy.setVolume(fileID, 1)
            self.audio_proxy.play(fileID)
        except Exception as e:
            self.session.reconnect()
            self.audio_proxy = self.session.get_service("ALAudioPlayer")
            fileID = self.audio_proxy.loadFile("/home/nao/alert.mp3")
            self.audio_proxy.setVolume(fileID, 1)
            self.audio_proxy.play(fileID)
        return "ACK"

    def callback_talking(self, value):
        """
        Records changes on the topic to keep track of the operation of the Text-to-Speech node.
        """
        self.talking = value.data


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    node_audio = AlertNode(options.ip, int(options.port))
    scheduler = BackgroundScheduler()
    running_job = scheduler.add_job(node_audio.alert, 'interval', seconds=2, max_instances=1)
    scheduler.start()
    rospy.init_node("audio_node", anonymous=True)
    rospy.Subscriber('pepper_say', Bool, node_audio.callback_talking)
    rospy.spin()