from __future__ import annotations
import pickle
from typing import Dict, List, Sequence, Union
from Task import Task

class CustomUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if name == 'ToDo':
            return ToDo
        return super().find_class(module, name)

class ToDo:
    def __init__(self) -> None:
        self._todo: Dict[str, List[Task]] = {}

    def _store(self, store_path="./todo.pickle"):
        with open(store_path, "wb") as out:
            pickle.dump(self, out)

    @staticmethod
    def load(store_path="./todo.pickle") -> ToDo:
        return CustomUnpickler(open(store_path, "rb")).load()

    def _is_task_in_category(self, category, tag):
        return any([task.tag==tag for task in self._todo[category]])

    def add_task(self, category, tag, deadline, alarm=None) -> bool:
        if category in self._todo:
            if self._is_task_in_category(category, tag):
                return False
            self._todo[category].append(Task(tag, deadline, alarm))
        else:
            self._todo[category] = [Task(tag, deadline, alarm)]
        return True

    def remove_task(self, category, tag) -> bool:
        if category in self._todo:
            if self._is_task_in_category(category, tag):
                self._todo[category] = [task for task in self._todo[category] if task.tag != tag]
                return True
        return False

    def modify_task(self, category, tag, tag_new=None, deadline_new=None, alarm_new=None) -> bool:
        task_old = self.get_task(category, tag)
        if task_old is None:
            return False # <-- caso in cui non vi sia la task da modificare

        self.remove_task(category, tag)
        self.add_task(category, tag_new if tag_new is not None else task_old.tag,
                                deadline_new if deadline_new is not None else task_old.deadline, 
                                alarm_new if alarm_new is not None else task_old.alarm)
        return True

    def move_task(self, category, tag, category_new) -> bool:
        if not category in self._todo or category_new in self._todo:
            return False

        task_old = self.get_task(category, tag)
        if task_old is None:
            return False

        self.remove_task(category, tag)
        self.add_task(category_new, tag, task_old.deadline, task_old.alarm)
        return True
        
    def get_tasks_of_category(self, category) -> Union[List, None]:
        if not category in self._todo:
            return None
        return self._todo[category]

    def get_tasks(self) -> Union[Dict, None]:
        categories = self._todo.keys()
        if not categories: # <-- gestisci il caso in cui non vi sia alcuna categoria
            return None

        tasks = {} # <-- gestisci anche il caso in cui non vi sia alcuna task
        for category in categories:
            t = self.get_tasks_of_category(category)
            if t:
                tasks[category] = t
        return tasks

    def get_categories(self) -> Union[List, None]:
        return list(self._todo.keys()) # <-- gestisci il caso in cui non vi sia alcuna categoria

    def remove_category(self, category) -> bool:
        if not category in self._todo:
            return False
        self._todo.pop(category)
        return True

    def get_task(self, category, tag) -> Union[Task, None]:
        if category not in self._todo:
            return None # <-- nel caso non vi sia una task con quella categoria (indistinguibile dal None successivo)

        tasks = [task for task in self._todo[category] if task.tag==tag]
        if tasks:
            return tasks[0]
        return None # <-- nel caso non vi sia una task con quel tag


if __name__ == "__main__":
    ToDo()._store()