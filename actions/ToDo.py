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

    def get_tasks_of_category(self, category) -> Union[List, None]:
        if not category in self._todo:
            return None
        return self._todo[category]

    def get_tasks(self) -> Union[List, None]:
        categories = self._todo.keys()
        if not categories: # <-- gestisci il caso in cui non vi sia alcuna categoria
            return None

        tasks = [] # <-- gestisci anche il caso in cui non vi sia alcuna task
        for category in categories:
            tasks += self._todo[category]
        return tasks

    def get_categories(self) -> Union[List, None]:
        return list(self._todo.keys()) # <-- gestisci il caso in cui non vi sia alcuna categoria


if __name__ == "__main__":
    ToDo()._store()