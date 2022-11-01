from typing import Any, Sequence, Tuple
from rasa_sdk import Tracker

def get_tag(tracker: Tracker) -> Any:
    return tracker.get_slot("tag")

def get_tag_new(tracker: Tracker) -> Any:
    return tracker.get_slot("tag_new")

def get_categories(tracker: Tracker) -> Any:
    return tracker.get_slot("category")

def get_deadline(tracker: Tracker) -> Any:
    date = tracker.get_slot("date")
    time = tracker.get_slot("time")

    if date is None or time is None:
        raise Exception
    return date + " " + time

def get_alert(tracker: Tracker) -> Any:
    return tracker.get_slot("alert")

def get_info(tracker: Tracker) -> Tuple[Any, Any, Any]:
    return get_tag(tracker), get_categories(tracker), get_deadline(tracker)

def sequence_to_str(seq: Sequence) -> str:
    return ", ".join(seq)

class TrackerFake:
    def __init__(self) -> None:
        self.slots = {}

    def get_slot(self, slot):
        return self.slots[slot]

class CollectingDispatcherFake:
    def __init__(self) -> None:
        pass

    def utter_message(self, text=""):
        print("Dispatched: ", text)