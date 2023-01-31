from __future__ import annotations
from typing import Dict, List
from actions.Task import Task
from actions.ActionsException import ExceptionMissingCategory, ExceptionMissingTask, ExceptionNoCategories, ExceptionNoTasks, ExceptionTaskExists, ExceptionMissingDeadline
import os
import datetime
import json


class ToDo:
    ##########################################################################
    # Init                                                                   #
    ##########################################################################
    def __init__(self) -> None:
        self.loaded_user = ""
        self._todo: Dict[str, List[Task]] = {}

    ##########################################################################
    # Store and load                                                         #
    ##########################################################################
    def store(self, user):
        with open(f"./todo_{user}.json", "w") as out:
            _todo = {}
            for cat in self._todo:
                _todo[cat] = [x.encode() for x in self._todo[cat]]
            json.dump(_todo, out, indent=4)

    @staticmethod
    def create_user(user: str) -> None:
        ToDo().store(user)

    @staticmethod
    def load(user: str) -> ToDo:
        with open(f"./todo_{user}.json", "r") as f:
            _todo = json.load(f)
        for cat in _todo:
            _todo[cat] = [Task.decode(x) for x in _todo[cat]]
        todo = ToDo()
        todo._todo = _todo
        todo.set_loaded_user(user)
        return todo

    def set_loaded_user(self, user: str) -> None:
        self.loaded_user = user

    def get_loaded_user(self) -> str:
        return self.loaded_user

    @staticmethod
    def is_user_available(user: str) -> bool:
        user_path = f"./todo_{user}.json"
        return os.path.exists(user_path)

    @staticmethod
    def remove_user(user: str) -> None:
        user_path = f"./todo_{user}.json"
        os.remove(user_path)

    ##########################################################################
    # Checks                                                                 #
    ##########################################################################
    def _check_category(self, category) -> None:
        if category not in self._todo:
            raise ExceptionMissingCategory(category)

    def _check_task(self, category, tag) -> None:
        self._check_category(category)
        if not any([task.tag==tag for task in self._todo[category]]):
            raise ExceptionMissingTask(category, tag)

    def _check_task_not_exists(self, category, tag) -> None:
        if category not in self._todo:
            return
        if any([task.tag==tag for task in self._todo[category]]):
            raise ExceptionTaskExists(category, tag)

    def _check_no_categories(self) -> None:
        if not self._todo.keys():
            raise ExceptionNoCategories()

    def _check_no_tasks(self) -> None:
        self._check_no_categories()
        for category in self._todo.keys():
            if self._todo[category]:
                return
        raise ExceptionNoTasks()

    def _check_no_deadline(self, category, tag):
        if self.get_task(category=category, tag=tag).deadline is None:
            raise ExceptionMissingDeadline()

    ##########################################################################
    # Tasks                                                                  #
    ##########################################################################
    def get_task(self, category: str, tag: str) -> Task:
        self._check_task(category, tag)
        return [task for task in self._todo[category] if task.tag==tag][0]

    def add_task(self, category: str, tag: str, deadline: datetime.datetime, alarm: datetime.datetime=None) -> None:
        if category in self._todo:
            self._check_task_not_exists(category, tag)
            self._todo[category].append(Task(tag, deadline, alarm))
        else:
            self._todo[category] = [Task(tag, deadline, alarm)]
        self.store(self.get_loaded_user())

    def remove_deadline(self, category: str, tag: str):
        self._check_no_deadline(category, tag)
        t = self.get_task(category=category, tag=tag)
        t.remove_deadline()
        t.remove_alarm()
        self.store(self.get_loaded_user())

    def remove_task(self, category: str, tag: str) -> None:
        self._check_task(category, tag)
        self._todo[category] = [task for task in self._todo[category] if task.tag != tag]
        self.store(self.get_loaded_user())

    def modify_task(self, category: str, tag: str, tag_new: str=None, deadline_new: datetime.datetime=None, alarm_new: datetime.datetime=None) -> None:
        self._check_task(category, tag)

        task_old = self.get_task(category, tag)
        self.remove_task(category, tag)
        self.add_task(category, tag_new if tag_new is not None else task_old.tag,
                                deadline_new if deadline_new is not None else task_old.deadline, 
                                alarm_new if alarm_new is not None else task_old.alarm)

    def remove_task_alarm(self, category: str, tag: str) -> None:
        self._check_task(category, tag)

        task_old = self.get_task(category, tag)
        self.remove_task(category, tag)
        self.add_task(category, task_old.tag,
                                task_old.deadline, 
                                None)

    def move_task(self, category: str, tag: str, category_new: str) -> None:
        self._check_task(category, tag)
        task_old = self.get_task(category, tag)
        self.remove_task(category, tag)
        self.add_task(category_new, tag, task_old.deadline, task_old.alarm)

    def get_tasks(self) -> Dict:
        self._check_no_categories()
        self._check_no_tasks()

        tasks = {}
        for category in self._todo.keys():
            t = self.get_tasks_of_category(category)
            if t:
                tasks[category] = t
        return tasks

    def get_tasks_of_category(self, category: str) -> List[Task]:
        self._check_category(category)
        return self._todo[category]

    ##########################################################################
    # Categories                                                             #
    ##########################################################################
    def get_categories(self) -> List:
        self._check_no_categories()
        return list(self._todo.keys())

    def remove_category(self, category) -> None:
        self._check_category(category)
        self._todo.pop(category)
        self.store(self.get_loaded_user())

    ##########################################################################
    # Utils                                                                  #
    ##########################################################################
    def clear_all(self) -> None:
        self._todo: Dict[str, List[Task]] = {}
        self.store(self.get_loaded_user())