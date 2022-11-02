from __future__ import annotations
import pickle
from typing import Dict, List
from actions.Task import Task
from actions.ActionsException import ExceptionMissingCategory, ExceptionMissingTask, ExceptionNoCategories, ExceptionNoTasks, ExceptionTaskExists


class CustomUnpickler(pickle.Unpickler):
    """Custom unpickler is used to automatically fix the pathing of the default pickle class"""
    def find_class(self, module, name):
        if name == 'ToDo':
            return ToDo
        return super().find_class(module, name)


class ToDo:
    latest_path = ""
    ##########################################################################
    # Init                                                                   #
    ##########################################################################
    def __init__(self) -> None:
        self._todo: Dict[str, List[Task]] = {}

    ##########################################################################
    # Store and load                                                         #
    ##########################################################################
    def _store(self, store_path="./todo.pickle"):
        with open(store_path, "wb") as out:
            pickle.dump(self, out)

    @staticmethod
    def load(store_path="./todo.pickle") -> ToDo:
        ToDo.latest_path = store_path
        return CustomUnpickler(open(store_path, "rb")).load()

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

    ##########################################################################
    # Tasks                                                                  #
    ##########################################################################
    def get_task(self, category, tag) -> Task:
        self._check_task(category, tag)
        return [task for task in self._todo[category] if task.tag==tag][0]

    def add_task(self, category, tag, deadline, alarm=None) -> None:
        if category in self._todo:
            self._check_task_not_exists(category, tag)
            self._todo[category].append(Task(tag, deadline, alarm))
        else:
            self._todo[category] = [Task(tag, deadline, alarm)]

    def remove_task(self, category, tag) -> None:
        self._check_task(category, tag)
        self._todo[category] = [task for task in self._todo[category] if task.tag != tag]

    def modify_task(self, category, tag, tag_new=None, deadline_new=None, alarm_new=None) -> None:
        self._check_task(category, tag)

        task_old = self.get_task(category, tag)
        self.remove_task(category, tag)
        self.add_task(category, tag_new if tag_new is not None else task_old.tag,
                                deadline_new if deadline_new is not None else task_old.deadline, 
                                alarm_new if alarm_new is not None else task_old.alarm)

    def move_task(self, category, tag, category_new) -> None:
        self._check_category(category_new)
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

    def get_tasks_of_category(self, category) -> List[Task]:
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

    ##########################################################################
    # Utils                                                                  #
    ##########################################################################
    def clear_all(self) -> None:
        self._todo: Dict[str, List[Task]] = {}

if __name__ == "__main__":
    # Run this file if you want to store a blank ToDo list
    ToDo()._store()