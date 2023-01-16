#!/usr/bin/python3
import os
from tablet.actions.ToDo import CustomUnpickler, ToDo
from std_msgs.msg import Int16MultiArray, Bool
import datetime
import rospy
from std_msgs.msg import String
from optparse import OptionParser
from utils import Session


class AlertNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.audio_proxy = self.session.get_service("ALAudioPlayer")
    
    def alert(self):
        try:
            print("try")
            #import os
            #print(os.path.isfile("/home/francesca/Scrivania/ROS_todoList_chatbot/cogrob_ws/src/pepper_nodes/ringtone_1_46486.wav"))
            #print(self.audio_proxy.getInstalledSoundSetsList())
            #self.audio_proxy.playWebStream("https://www.youtube.com/watch?v=WfhLLBKdD5w&ab_channel=Melamarcia5535",0.2,0)
            fileID = self.audio_proxy.loadFile("/home/nao/alert.mp3")
            self.audio_proxy.setVolume(fileID, 1)
            self.audio_proxy.play(fileID)
            #fileID = self.audio_proxy.playFile("/home/nao/1.mp3")
        except Exception as e:
            print(e)
            self.session.reconnect()
            self.audio_proxy = self.session.get_service("ALAudioPlayer")
            fileID = self.audio_proxy.loadFile("/home/nao/alert.mp3")
            self.audio_proxy.setVolume(fileID, 1)
            self.audio_proxy.play(fileID)
        return "ACK"

def get_todo_data_path(path):
    tab_data = []
    todo_path = path
    todo: ToDo = CustomUnpickler(open(todo_path, "rb")).load()
    if todo._todo:
        for c in todo.get_categories():
            tasks = todo.get_tasks_of_category(c)
            for t in tasks:
                tab_data.append((c, t.tag, t.deadline if t.deadline is not None else "", 
                                    t.alarm))
    return tab_data

def check_alerts(todo_dir, alert_length: int=15):
    data_with_alerts = []
    todo_paths = [x for x in os.listdir(todo_dir) if x.endswith(".pickle")]
    for todo_path in todo_paths:
        data = get_todo_data_path(todo_dir + todo_path)
        for d in data:
            if d[-1] is not None and datetime.datetime.now() >= d[-1] and datetime.datetime.now() <= d[-1] + datetime.timedelta(seconds=alert_length):
                data_with_alerts.append(d)
    return data_with_alerts

def callback(value):
    actual_alerts = check_alerts("../../../chatbot/")
    if value.data == False and len(actual_alerts)>0:
        print("CI SONOOOOOOO")
        node_audio.alert()
    
if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()
    node_audio = AlertNode(options.ip, int(options.port))
    rospy.init_node("audio_node", anonymous=True)
    rospy.Subscriber('pepper_say', Bool, callback)
    rospy.spin()