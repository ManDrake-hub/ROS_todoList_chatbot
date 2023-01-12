import datetime
from typing import Any, Sequence, Tuple, Dict, List, Any
from rasa_sdk import Tracker
from actions.Task import Task
from actions.ToDo import ToDo
from actions.ActionsException import ExceptionNoCategories, ExceptionDateTimeBeforeNow, ExceptionDateTimeFormatInvalid
from dateutil.parser import parse
import re


def get_user(tracker: Tracker) -> Any:
    return tracker.get_slot("user")

def get_user_new(tracker: Tracker) -> Any:
    return tracker.get_slot("user_new")

def get_tag(tracker: Tracker) -> Any:
    return tracker.get_slot("tag")

def get_tag_new(tracker: Tracker) -> Any:
    return tracker.get_slot("tag_new")

def get_category(tracker: Tracker) -> Any:
    return tracker.get_slot("category")

def get_category_new(tracker: Tracker) -> Any:
    return tracker.get_slot("category_new")

def _is_slot_empty(tracker: Tracker, slot: str) -> bool:
    try:
        _slot = tracker.get_slot(slot)
        return (isinstance(_slot, list) and len(_slot) == 0) or _slot is None
    except:
        return True
    
def get_logical_alert(tracker: Tracker) -> Any:
    return tracker.get_slot("logical_alert")

def get_deadline(tracker: Tracker) -> Any:
    if (_is_slot_empty(tracker, "date") or 
        _is_slot_empty(tracker, "time")):
        return None

    date = tracker.get_slot("date")
    time = tracker.get_slot("time")
    try:
        dt = convert_deadline_to_datetime(date, time)
    except:
        raise ExceptionDateTimeFormatInvalid()
    if is_datetime_before_now(dt):
        raise ExceptionDateTimeBeforeNow()
    return dt

def get_alert(tracker: Tracker) -> Any:
    return tracker.get_slot("alert")

def get_info(tracker: Tracker) -> Tuple[Any, Any, Any]:
    return get_tag(tracker), get_category(tracker), get_deadline(tracker)

def sequence_to_str(seq: Sequence) -> str:
    return ", ".join(seq)

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

"""
Casi testati in cui funziona:

- [20/11/1969](date)
- [12-10-1999](date)
- [30-06-65](date)
- [15/03/87](date)
- [12-3-29](date)
- [6-1-99](date)
- [7/8/99](date)
- [13-8-2004](date)
- [4/8/2003](date)
- [4.8.2003](date)
- [20.11.1969](date)
- [04:25](time)
- [7:30](time)
- [7:7](time)
- [23:59](time)
- [20:7](time)
"""
def next_weekday(d, weekday):
    days_ahead = weekday - d.weekday()
    if days_ahead <= 0: # Target day already happened this week
        days_ahead += 7
    return d + datetime.timedelta(days_ahead)

def convert_deadline_to_datetime(date: str, time: str) -> datetime.datetime:
    weekdays = ["lunedì", "martedì", "mercoledì", "giovedì", "venerdì", "sabato", "domenica"]
    offsets = ["oggi", "domani", "dopodomani"]
    if re.search('[a-zA-Z]', date) is not None and re.search('[0-9]', date) is None:
        date = date.lower()
        if any([x in date for x in weekdays]):
            date = next_weekday(datetime.datetime.today(), weekday=[x in date for x in weekdays].index(True))
        elif any([x in date for x in offsets]):
            date = datetime.datetime.today() + datetime.timedelta(days=[x in date for x in offsets].index(True))
        else:
            raise Exception("Could not parse date")
    elif re.search('[0-9]', date) is not None and len(re.findall(r'\d+', date)) == 1:
        months = ["gennaio", "febbraio", "marzo", "aprile", "maggio", "giugno", "luglio", "agosto", "settembre", "ottobre", "novembre", "dicembre"]
        date = datetime.datetime(year=datetime.date.today().year, 
                            month=([x in date for x in months].index(True) + 1) if (re.search('[a-zA-Z]', date) is not None) else datetime.datetime.today().month, 
                            day=int(re.findall(r'\d+', date)[0]))

    if date == "" or date is None or type(date) is datetime.datetime:
        dt = parse(time)
        if dt < datetime.datetime.now():
            return parse(time, default=((datetime.datetime.now() + datetime.timedelta(days=1)) if type(date) is not datetime.datetime else date))
        return dt # ?
    return parse(date + " " + time, dayfirst=True)

def is_datetime_before_now(dt: datetime.datetime) -> bool:
    return dt < datetime.datetime.now()