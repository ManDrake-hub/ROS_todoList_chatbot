import datetime
from typing import Dict, List, Any
from actions.actions import ActionRemoveDeadline, ActionRenameUser, ActionWrapper, ActionAddTask, ActionRemoveTask, ActionMoveTask, ActionCreateUser, ActionSetUser, ActionRemoveUser, ActionGetUser, ActionReadTasks, ActionAddAlert, ActionRemoveAlert
from actions.Task import Task
from actions.utils_test import check_equals, print_todo, print_todo_dict, CollectingDispatcherFake, TrackerFake, test_action, ask_rasa
from actions.ToDo import ToDo
from actions.utils import convert_deadline_to_datetime
import requests
import os
import unittest
from actions.ActionsException import ExceptionMissingTask, ExceptionMissingCategory, ExceptionNoCategories

"""
class RasaTest(unittest.TestCase):
    # Before runnning the test, run in different terminals:
    # 1) rasa run -m models --endpoints endpoints.yml --port 5002 --credentials credentials.yml --enable-api
    # 2) rasa run actions
    def __init__(self, methodName: str = ...) -> None:
        super().__init__(methodName)
        CollectingDispatcherFake.debug = False

    def setUp(self):
        ##########################################################
        # Setup for rasa tests
        # Create empty todo list
        if os.path.exists("./todo_test.json"):
            os.remove("./todo_test.json")
        # Start conversation
        ask_rasa("Ciao")
        ask_rasa("Sono test")
        ##########################################################

    def test_insert(self):
        # Test insert
        ask_rasa("Aggiungi l'attività fagioli nella categoria spesa")
        ask_rasa("no")
        todo = ToDo.load("test")
        todo.get_task("spesa", "fagioli")

    def test_insert_with_deadline(self):
        # Test insert with deadline
        ask_rasa("Aggiungi l'attività saltare nella categoria palestra")
        ask_rasa("si")
        ask_rasa("domani alle ore 23:00")
        ask_rasa("no")
        todo = ToDo.load("test")
        task = todo.get_task("palestra", "saltare")
        tomorrow = datetime.datetime.now() + datetime.timedelta(days=1)
        assert (task.deadline.year == tomorrow.year and task.deadline.month == tomorrow.month and task.deadline.day == tomorrow.day)
        assert (task.deadline.hour == 23 and task.deadline.minute == 0)

    def test_insert_with_deadline_and_alert(self):
        # Test insert with deadline and alert
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("si")
        ask_rasa("dopodomani alle ore 23:00")
        ask_rasa("si")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        tomorrow = datetime.datetime.now() + datetime.timedelta(days=2)
        assert (task.deadline.year == tomorrow.year and task.deadline.month == tomorrow.month and task.deadline.day == tomorrow.day)
        assert (task.deadline.hour == 23 and task.deadline.minute == 0)

    def test_remove_alert(self):
        # Test remove alert
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("si")
        ask_rasa("dopodomani alle ore 23:00")
        ask_rasa("si")
        ask_rasa("Rimuovi l'avviso dell'attività tastiera nella categoria scrivania")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        assert task.alarm is None

    def test_remove_action(self):
        # Test remove action
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("si")
        ask_rasa("dopodomani alle ore 23:00")
        ask_rasa("si")
        ask_rasa("Rimuovi l'attività tastiera dalla categoria scrivania")
        todo = ToDo.load("test")
        self.assertRaises(ExceptionMissingTask, todo.get_task, "scrivania", "tastiera")

    def test_add_deadline(self):
        # Test add deadline
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("no")
        ask_rasa("Aggiungi la deadline")
        ask_rasa("domani alle ore 23:00")
        ask_rasa("attività tastiera")
        ask_rasa("categoria scrivania")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        tomorrow = datetime.datetime.now() + datetime.timedelta(days=1)
        assert (task.deadline.year == tomorrow.year and task.deadline.month == tomorrow.month and task.deadline.day == tomorrow.day)
        assert (task.deadline.hour == 23 and task.deadline.minute == 0)

    def test_add_alert(self):
        # Test add alert 
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("no")
        ask_rasa("Aggiungi la deadline")
        ask_rasa("domani alle ore 23:00")
        ask_rasa("attività tastiera")
        ask_rasa("categoria scrivania")
        ask_rasa("Aggiungi un alert")
        ask_rasa("attività tastiera")
        ask_rasa("categoria scrivania")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        assert task.alarm is not None

    def test_remove_deadline_with_alert_present(self):
        # Test remove deadline if the alert is present
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("si")
        ask_rasa("dopodomani alle ore 23:00")
        ask_rasa("si")
        ask_rasa("Rimuovi la deadline per l'attività tastiera nella categoria scrivania")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        assert task.deadline is None and task.alarm is None

    def test_add_alert_without_deadline(self):
        # Test add alert without deadline
        ask_rasa("Aggiungi l'attività tastiera nella categoria scrivania")
        ask_rasa("no")
        ask_rasa("Aggiungi un alert")
        ask_rasa("attività tastiera")
        ask_rasa("categoria scrivania")
        todo = ToDo.load("test")
        task = todo.get_task("scrivania", "tastiera")
        assert task.deadline is None and task.alarm is None

    def test_remove_category(self):
        # Test remove category
        ask_rasa("Aggiungi l'attività fagioli nella categoria spesa")
        ask_rasa("no")
        ask_rasa("Aggiungi l'attività pane nella categoria spesa")
        ask_rasa("no")
        ask_rasa("Voglio rimuovere la categoria spesa")
        todo = ToDo.load("test")
        self.assertRaises(ExceptionNoCategories, todo.get_categories)

    def test_modify_tag(self):
        # Test modify tag
        ask_rasa("Aggiungi l'attività fagioli nella categoria spesa")
        ask_rasa("no")
        ask_rasa("Voglio modificare un'attività")
        ask_rasa("attività fagioli")
        ask_rasa("categoria spesa")
        ask_rasa("nuovo titolo pane")
        todo = ToDo.load("test")
        self.assertRaises(ExceptionMissingTask, todo.get_task, "spesa", "fagioli")
        todo.get_task("spesa", "pane")

    def test_modify_category(self):
        # Test modify tag
        ask_rasa("Aggiungi l'attività fagioli nella categoria spesa")
        ask_rasa("no")
        print(ask_rasa("Voglio spostare l'attività fagioli in categoria spesa in cena"))
        print(ask_rasa("categoria cena"))
        todo = ToDo.load("test")
        self.assertRaises(ExceptionMissingTask, todo.get_task, "spesa", "fagioli")
        todo.get_task("cena", "fagioli")
"""


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
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": "10/10/2023 10:10:10"}, 
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
        test_action(todo, ActionAddTask, {"category": "spesa", "tag": "fagioli", "date": "10/10/2023", "time": "10:10:10", "logical_alert": None}, 
                                        {"spesa": [Task("fagioli", self.test_datetime), ]}, check=True)
        test_action(todo, ActionRemoveAlert, {"category": "spesa", "tag": "fagioli"}, {"spesa": []}, check=True)