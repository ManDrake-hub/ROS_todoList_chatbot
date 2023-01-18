import datetime
from typing import Dict, List, Any
from actions.actions import ActionModifyTaskName, ActionWrapper, ActionSetUser, Action, ActionAddAlert, ActionAddTask, ActionCreateUser, ActionGetUser, ActionModifyAlert, ActionModifyDeadline, \
                            ActionMoveModifyTask, ActionMoveTask, ActionReadCategories, ActionReadCategory, ActionReadTask, ActionReadTasks, ActionRemoveAlert, ActionRemoveCategory, ActionRemoveDeadline, \
                            ActionRemoveTask, ActionRemoveUser, ActionRenameUser, ActionReset, ActionSet
from actions.Task import Task
from utils_test import check_equals, print_todo, print_todo_dict, CollectingDispatcherFake, TrackerFake, test_action, ask_rasa
from actions.ToDo import ToDo
from actions.utils import convert_deadline_to_datetime
import requests
import os
import unittest
from actions.ActionsException import ExceptionMissingTask, ExceptionMissingCategory, ExceptionNoCategories


# To run the tests: python -m unittest discover
class ActionsTest(unittest.TestCase):
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)
        CollectingDispatcherFake.debug = False
        self.test_datetime = datetime.datetime(year=2023, month=10, day=10, hour=10, minute=10, second=10)

    def get_clear_todo(self):
        ##########################################################
        # Setup for action tests
        # Create empty todo list
        if os.path.exists("./todo_test_actions.json"):
            os.remove("./todo_test_actions.json")
        ToDo.create_user("test_actions")
        return ToDo.load("test_actions")
        ##########################################################

    def test_action_insert(self):
        # Test action insert
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": None, "time": None, "logical_alert": None}, 
                                        {"spesa": [Task("fagioli"), ]})

    def test_action_insert_with_deadline(self):
        # Test action insert with deadline
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]})

    def test_action_insert_with_deadline_and_alarm(self):
        # Test action insert with deadline and alarm
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]})

    def test_remove_action(self):
        # Test remove action
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]}, check=True)
        test_action(todo, ActionRemoveTask, {"category": "spesa", "tag": "fagioli"}, {"spesa": []}, check=True)

    def test_remove_alert(self):
        # Test remove alert
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)
        test_action(todo, ActionRemoveAlert, {"category": "spesa", "tag": "fagioli"}, {"spesa": [Task("fagioli", self.test_datetime), ]}, check=True)

    def test_remove_alert(self):
        # Test remove alert
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)
        test_action(todo, ActionRemoveAlert, {"category": "spesa", "tag": "fagioli"}, {"spesa": [Task("fagioli", self.test_datetime), ]}, check=True)

    def test_action_insert_deadline(self):
        # Test action insert deadline
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": None, "time": None, "logical_alert": None}, 
                                        {"spesa": [Task("fagioli"), ]})
        test_action(todo, ActionModifyDeadline, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]})

    def test_action_insert_alert(self):
        # Test action insert alert
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]})
        test_action(todo, ActionAddAlert, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]})

    def test_action_remove_deadline_with_alert_present(self):
        # Test action remove deadline with alert present
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]})
        test_action(todo, ActionRemoveDeadline, {"category": "spesa", "tag": "fagioli"}, 
                                        {"spesa": [Task("fagioli") ]})

    def test_action_add_alert_without_deadline(self):
        # Test action add alert without deadline
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": None, "time": None, "logical_alert": None}, 
                                        {"spesa": [Task("fagioli"), ]})

        test_action(todo, ActionAddAlert, {"category": "spesa", "tag": "fagioli"}, 
                                        {"spesa": [Task("fagioli"), ]})

    def test_remove_category(self):
        # Test remove category
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)
        test_action(todo, ActionRemoveCategory, {"category": "spesa"}, {}, check=True)

    def test_modify_tag(self):
        # Test modify tag
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)
        test_action(todo, ActionModifyTaskName, {"category": "spesa", "tag": "fagioli", "tag_new": "pane"}, 
                                        {"spesa": [Task("pane", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)

    def test_modify_category(self):
        # Test modify category
        todo = self.get_clear_todo()
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": True}, 
                                        {"spesa": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=True)
        test_action(todo, ActionMoveTask, {"category": "spesa", "tag": "fagioli", "category_new": "cena"}, 
                                        {"cena": [Task("fagioli", self.test_datetime, self.test_datetime-datetime.timedelta(minutes=5)), ]}, check=False)