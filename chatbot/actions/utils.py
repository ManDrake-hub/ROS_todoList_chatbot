import datetime
from typing import Any, Sequence, Tuple, Dict, List, Any, Union
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

def next_weekday(d, weekday):
    days_ahead = weekday - d.weekday()
    if days_ahead <= 0: # Target day already happened this week
        days_ahead += 7
    return d + datetime.timedelta(days_ahead)

def has_letters(text: str) -> bool:
    return re.search('[a-zA-Z]', text) is not None

def has_numbers(text: str) -> bool:
    return re.search('[0-9]', text) is not None

def list_of_numbers(text: str) -> List:
    return re.findall(r'\d+', text)

def has_element(text: str, l: List) -> Tuple[bool, int]:
    _has_element = any([x in text for x in l])
    if _has_element:
        return _has_element, [i for i, x in enumerate(l) if x in text][-1]# [x in text for x in l].index(True)
    return _has_element, 0

def is_datetime_before_now(dt: datetime.datetime) -> bool:
    return dt < datetime.datetime.now()

def convert_deadline_to_datetime(date: str, time: str) -> datetime.datetime:
    """
    Convert a date and time provided as string to a datetime.

    Supports both normal formats like 10/01/2025 but also:
    - day + time: ex. "lunedì alle 10:10:10"
    - day + month + time: ex. "27 gennaio alle 10:10:10"
    - offset + time: ex. "oggi alle 10:10:10"

    If time is an empty string, the current time will be used, if date is an empty string the current day will be used
    or the next day if it would lead to a datetime already passed.
    """
    date = date.lower()
    weekdays = ["lunedì", "martedì", "mercoledì", "giovedì", "venerdì", "sabato", "domenica"]
    months = ["gennaio", "febbraio", "marzo", "aprile", "maggio", "giugno", "luglio", "agosto", "settembre", "ottobre", "novembre", "dicembre"]
    offsets = ["oggi", "domani", "dopodomani"]

    # We check for weekdays, months and offsets inside the given date
    has_weekdays, weekdays_index = has_element(date, weekdays)
    has_months, months_index = has_element(date, months)
    has_offsets, offsets_index = has_element(date, offsets)

    # If has letters and not numbers, it can be an offset or a weekday
    if has_letters(date) and not has_numbers(date):
        if has_weekdays:
            date = next_weekday(datetime.datetime.today(), weekday=weekdays_index)
        elif has_offsets:
            date = datetime.datetime.today() + datetime.timedelta(days=offsets_index)
        else:
            raise Exception("Could not parse date")

    # If it has only one number, it can be a day with or without its month specified in letters
    elif has_numbers(date) and len(list_of_numbers(date)) == 1:
        date = datetime.datetime(year=datetime.date.today().year, 
                                month=months_index+1 if has_months else datetime.datetime.today().month, 
                                day=int(list_of_numbers(date)[0]))
    # If it has two numbers, it can be a day with or without its month specified in letters and the year
    elif has_numbers(date) and len(list_of_numbers(date)) == 2:
        date = datetime.datetime(year=int(list_of_numbers(date)[-1]),
                                month=months_index+1 if (has_letters(date)) else datetime.datetime.today().month, 
                                day=int(list_of_numbers(date)[0]))
    if type(date) is datetime.datetime:
        return parse(time, default=date, dayfirst=True)

    if date == "" or date is None:
        # If the date has not been specified, we parse the time with the current date as default
        dt = parse(time, dayfirst=True)
        if is_datetime_before_now(dt):
            # If with the current date, the datetime has been passed, we reparse it with a day added 
            return parse(time, default=(datetime.datetime.now() + datetime.timedelta(days=1)), dayfirst=True)
        return dt
    # If both the date and the time has been specified (and the date has not been converted using prev converters)
    # Parse using both date and time specified
    return parse(date + " " + time, dayfirst=True)