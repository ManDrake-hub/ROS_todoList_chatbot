from typing import Any, Sequence, Tuple, Dict, List, Any
from rasa_sdk import Tracker
from Task import Task
from ToDo import ToDo

def get_tag(tracker: Tracker) -> Any:
    return tracker.get_slot("tag")

def get_tag_new(tracker: Tracker) -> Any:
    return tracker.get_slot("tag_new")

def get_category(tracker: Tracker) -> Any:
    return tracker.get_slot("category")

def get_category_new(tracker: Tracker) -> Any:
    return tracker.get_slot("category_new")

def get_deadline(tracker: Tracker) -> Any:
    date = tracker.get_slot("date")
    time = tracker.get_slot("time")

    if date is None or time is None:
        raise Exception
    return date + " " + time

def get_alert(tracker: Tracker) -> Any:
    return tracker.get_slot("alert")

def get_info(tracker: Tracker) -> Tuple[Any, Any, Any]:
    return get_tag(tracker), get_category(tracker), get_deadline(tracker)

def sequence_to_str(seq: Sequence) -> str:
    return ", ".join(seq)

def print_todo(todo: ToDo):
    print("These are the categories: ", list(todo.get_categories()))

    for k in todo.get_categories():
        print("This are the tasks for: ")
        print(f"    Category: {k}")
        for t in todo.get_tasks_of_category(k):
            print(f"         - {str(t)}")

def print_todo_dict(todo: Dict):
    print("These are the categories: ", list(todo.keys()))

    for k in todo.keys():
        print("This are the tasks for: ")
        print(f"    Category: {k}")
        for t in todo[k]:
            print(f"         - {str(t)}")

def check_equals_task(real_task: Task, expected_task: Task):
    return real_task.tag == expected_task.tag and real_task.deadline == expected_task.deadline and real_task.alarm == expected_task.alarm

def check_equals(real: ToDo, expected: Dict[str, List[Task]]):
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