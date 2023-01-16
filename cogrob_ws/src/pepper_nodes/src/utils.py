#!/usr/bin/python3
import qi
import sys

'''
This class creates a qi session using the ip and port parameters provided as input to costructor
'''
class Session:
    '''
    The costructor creates a qi session object. It then uses the ip and port parameters to connect the object to Pepper OS
    '''
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self._session = qi.Session()
        self._connect()
    
    '''
    This method uses the session object to connect to Pepper OS. If an exception occurs, the application is killed.
    '''
    def _connect(self):
        try:
            self.session.connect("tcp://" + self.ip + ":" + str(self.port))
        except RuntimeError:
            print("Can't connect to Naoqi at ip \"" + self.ip + "\" on port " + str(self.port) + ".\n "
                                                                                                 "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)

    '''
    This method reconnect the session object to Pepper OS
    @return: Returns the session object
    '''
    def reconnect(self):
        self._connect()
        return self.session
    
    '''
    Getter for the session object
    '''
    @property
    def session(self):
        return self._session
    
    '''
    This method returns the NaoQi service given as parameter.
    @param service_name: The name of the NaoQi service in form of string
    @return: Returns the service object
    '''
    def get_service(self, service_name: str):
        return self.session.service(service_name)

import pathlib
from dateutil import parser
import json
import os
import datetime
from typing import List, Tuple

def get_body(user):
    return f"""
            <head><h1>TODO list di {user}</h1></head>
            <body style="background-color:rgb(40, 42, 54);">
            """
def get_body_closure():
    return """</body>"""

def read_user(path="../../../../../.ros/name.txt")-> str:
    """Read user name from given file path"""
    with open(path, "r") as f:
        return f.read().split("\n")[0]

def get_todo_data(user, folder_json="../../../chatbot/") -> List[Tuple[str, str, str, str]]:
    """
    Creates a list of all tasks for the given user name.
    The list contains tuples filled as ("category", "tag", "deadline", "alarm").
    """
    todo_data = []

    if not os.path.exists(f"{folder_json}todo_{user}.json"):
        return todo_data
    
    _todo = json.load(open(f"{folder_json}todo_{user}.json", "r"))
    for cat in _todo:
        for t in _todo[cat]:
            task = json.loads(t)
            todo_data.append((cat, task["tag"], task["deadline"] if task["deadline"] is not None else "", 
                                task["alarm"] if task["alarm"] is not None else ""))
    return todo_data

def get_rows_from_data(data: List[Tuple[str, str, str, str]]):
    """Creates html table's rows from a given list of tuples filled as ("category", "tag", "deadline", "alarm")."""
    rows = []
    for d in data:
        rows.append("<tr>" + " ".join(["<td>" + str(x) + "</td>" for x in d[:-1]]) + "</tr>")
    return "\n".join(rows)

def check_alerts(folder_json="../../../chatbot/", alert_length: int=15):
    """
    Checks all todo lists for tasks that have alarm within "alert_length" from the current time.
    Returns a list of tuples filled as ("user", "category", "tag", "deadline", "alarm").
    """
    todo_data_with_alerts = []
    users = [x.replace(".json", "").split("_")[-1] for x in os.listdir(folder_json) if x.endswith(".json")]
    for user in users:
        data = get_todo_data(user, folder_json=folder_json)
        for d in data:
            if d[-1] == "":
                continue
            if d[-1] is not None and datetime.datetime.now() >= parser.parse(d[-1], dayfirst=True) and datetime.datetime.now() <= parser.parse(d[-1], dayfirst=True) + datetime.timedelta(seconds=alert_length):
                todo_data_with_alerts.append((user, *d))
    return todo_data_with_alerts
