import datetime
from typing import Any, Sequence, Tuple, Dict, List, Any
from rasa_sdk import Tracker
from actions.Task import Task
from actions.ToDo import ToDo
from actions.ActionsException import ExceptionNoCategories, ExceptionDateTimeBeforeNow, ExceptionDateTimeFormatInvalid
from dateutil.parser import parse
import re
from actions.actions import ActionWrapper
import requests


def ask_rasa(input_text: str) -> str:
    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }
    r = requests.post(get_answer_url, json=message)
    return r.json()[0]["text"]

def print_todo(todo: ToDo):
    try:
        if not todo.get_categories():
            return
    except ExceptionNoCategories:
        return

    print("These are the categories: ", list(todo.get_categories()))

    for k in todo.get_categories():
        print("This are the tasks for: ")
        print(f"    Category: {k}")
        for t in todo.get_tasks_of_category(k):
            print(f"         - {str(t)}")

def print_todo_dict(todo: Dict):
    try:
        if not todo.keys():
            return
    except ExceptionNoCategories:
        return

    print("These are the categories: ", list(todo.keys()))

    for k in todo.keys():
        print("This are the tasks for: ")
        print(f"    Category: {k}")
        for t in todo[k]:
            print(f"         - {str(t)}")

def check_equals_task(real_task: Task, expected_task: Task):
    return real_task.tag == expected_task.tag and real_task.deadline == expected_task.deadline and real_task.alarm == expected_task.alarm

def check_equals(real: ToDo, expected: Dict[str, List[Task]]):
    try:
        if len(real.get_categories()) == 0 and len(expected.keys()) == 0:
            return True
    except ExceptionNoCategories:
        if len(expected.keys()) == 0:
            return True
        return False

    for k in real.get_categories():
        if k not in expected.keys():
            return False
    for k in expected.keys():
        if k not in real.get_categories():
            return False

    for k in real.get_categories():
        for real_task in real.get_tasks_of_category(k):
            if not any([check_equals_task(real_task, expected_task) for expected_task in expected[k]]):
                return False
    return True

class CollectingDispatcherFake:
    debug = True

    @staticmethod
    def set_debug(debug):
        CollectingDispatcherFake.debug = debug

    @staticmethod
    def utter_message(text=""):
        if CollectingDispatcherFake.debug:
            print("Dispatched: ", f"'{text}'")

class TrackerFake:
    def __init__(self, slots: Dict) -> None:
        self._slots = slots

    def get_slot(self, slot: str):
        return self._slots[slot]

def test_action(todo: ToDo, action_to_test: ActionWrapper, slots: Dict[str, Any], todo_expected: Dict[str, List[Task]], clear_todo_before_running: bool=False, check=True):
    ActionWrapper.todo = todo
    if clear_todo_before_running:
        todo.clear_all()
    tracker_fake = TrackerFake(slots)
    action_to_test().run(CollectingDispatcherFake, tracker_fake, None)

    if check and not check_equals(todo, todo_expected):
        print_todo(todo)
        print_todo_dict(todo_expected)
        raise Exception("ToDo does not match the one expected")