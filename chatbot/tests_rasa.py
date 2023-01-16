import datetime
from typing import Dict, List, Any
from actions.actions import ActionRemoveDeadline, ActionRenameUser, ActionWrapper, ActionAddTask, ActionRemoveTask, ActionMoveTask, ActionCreateUser, ActionSetUser, ActionRemoveUser, ActionGetUser, ActionReadTasks, ActionAddAlert
from actions.Task import Task
from actions.utils_test import check_equals, print_todo, print_todo_dict, CollectingDispatcherFake, TrackerFake, test_action
from actions.ToDo import ToDo
from actions.utils import convert_deadline_to_datetime
import requests
import os


def ask_rasa(input_text: str) -> str:
    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }
    r = requests.post(get_answer_url, json=message)
    return r.json()[0]["text"]

def rasa_clean_start():
    ##########################################################
    # Setup for rasa tests
    # Create empty todo list
    if os.path.exists("./todo_test.json"):
        os.remove("./todo_test.json")
    # Start conversation
    answer = ask_rasa("Ciao")
    answer = ask_rasa("Sono test")
    ##########################################################

def actions_clean_start():
    ##########################################################
    # Setup for action tests
    # Create empty todo list
    if os.path.exists("./todo_test_actions.json"):
        os.remove("./todo_test_actions.json")
    ToDo.create_user("test_actions")
    return ToDo.load("test_actions")
    ##########################################################

if __name__ == "__main__":
    # Before runnning the test, run in a different terminal:
    # rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
    CollectingDispatcherFake.debug = False

    ##########################################################
    # Setup for rasa tests
    rasa_clean_start()
    ##########################################################

    # Test insert
    answer = ask_rasa("Aggiungi l'attività fagioli nella categoria spesa")
    answer = ask_rasa("no")
    todo = ToDo.load("test")
    todo.get_task("spesa", "fagioli")

    # Test insert with deadline
    answer = ask_rasa("Aggiungi l'attività saltare nella categoria palestra")
    answer = ask_rasa("si")
    answer = ask_rasa("domani alle ore 23:00")
    answer = ask_rasa("no")
    todo = ToDo.load("test")
    task = todo.get_task("palestra", "saltare")
    tomorrow = datetime.datetime.now() + datetime.timedelta(days=1)
    assert (task.deadline.year == tomorrow.year and task.deadline.month == tomorrow.month and task.deadline.day == tomorrow.day)
    assert (task.deadline.hour == 23 and task.deadline.minute == 0)

    # Test insert with deadline and alert
    answer = ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
    answer = ask_rasa("si")
    answer = ask_rasa("dopodomani alle ore 23:00")
    answer = ask_rasa("si")
    todo = ToDo.load("test")
    task = todo.get_task("scrivania", "tastiera")
    tomorrow = datetime.datetime.now() + datetime.timedelta(days=2)
    assert (task.deadline.year == tomorrow.year and task.deadline.month == tomorrow.month and task.deadline.day == tomorrow.day)
    assert (task.deadline.hour == 23 and task.deadline.minute == 0)

    # Test remove alert
    answer = ask_rasa("Rimuovi l'avviso dell'attività tastiera nella categoria scrivania")
    todo = ToDo.load("test")
    task = todo.get_task("scrivania", "tastiera")
    tomorrow = datetime.datetime.now() + datetime.timedelta(days=1)
    assert task.deadline == datetime.datetime(year=tomorrow.year, month=tomorrow.month, day=tomorrow.day, hour=23, minute=0, second=0)
    assert task.alarm is None

    
    test_datetime = datetime.datetime(year=2023, month=10, day=10, hour=10, minute=10, second=10)
    ##########################################################
    # Test action add
    todo = actions_clean_start()
    test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": False}, 
                                     {"spesa": [Task("fagioli", test_datetime), ]})
    # Test action remove
    test_action(todo, ActionRemoveTask, {"category": "spesa", "tag": "fagioli"}, {"spesa": []})

    print("Every test passed successfully!")