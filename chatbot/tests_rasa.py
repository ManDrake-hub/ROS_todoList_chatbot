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

if __name__ == "__main__":
    # Before runnning the test, run in a different terminal:
    # rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
    CollectingDispatcherFake.debug = False

    ##########################################################
    # Setup for rasa tests
    # Create empty todo list
    if os.path.exists("./todo_test.json"):
        os.remove("./todo_test.json")
    # Start conversation
    answer = ask_rasa("Ciao")
    answer = ask_rasa("Sono test")
    ##########################################################

    # Test insert
    answer = ask_rasa("Aggiungi l'attività casa nella categoria isolato")
    answer = ask_rasa("no")
    todo = ToDo.load("test")
    todo.get_task("isolato", "casa")

    ##########################################################
    # Setup for action tests
    # Create empty todo list
    if os.path.exists("./todo_test_actions.json"):
        os.remove("./todo_test_actions.json")
    ToDo.create_user("test_actions")
    ##########################################################

    # Test remove
    todo = ToDo.load("test_actions")
    base_datetime = datetime.datetime(year=2023, month=10, day=10, hour=10, minute=10, second=10)
    test_action(todo, ActionAddTask, {"category": "a", "tag": "remove", "date": "10/10/2023", "time": "10:10:10", "logical_alert": False}, 
                               {"a": [Task("remove", base_datetime), ]}, clear_todo_before_running=True)
    test_action(todo, ActionRemoveTask, {"category": "a", "tag": "remove"}, {"a": []}, clear_todo_before_running=False)

    print("Every test passed successfully!")