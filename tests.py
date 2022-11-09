import datetime
from typing import Dict, List, Any
from actions.actions import ActionRemoveDeadline, ActionRenameUser, ActionWrapper, ActionAddTask, ActionRemoveTask, ActionMoveTask, ActionCreateUser, ActionSetUser, ActionRemoveUser, ActionGetUser, ActionReadTasks, ActionAddAlert
from actions.Task import Task
from actions.utils import check_equals, print_todo, print_todo_dict
from actions.ToDo import ToDo

class CollectingDispatcherFake:
    debug = True

    @staticmethod
    def set_debug(debug):
        CollectingDispatcherFake.debug = debug

    @staticmethod
    def utter_message(text=""):
        if CollectingDispatcherFake.debug:
            print("Dispatched: ", f"'{text}'")

class TrackerFake:
    def __init__(self, slots: Dict) -> None:
        self._slots = slots

    def get_slot(self, slot: str):
        return self._slots[slot]

def test_action(action_to_test: ActionWrapper, slots: Dict[str, Any], todo_expected: Dict[str, List[Task]], clear: bool=True, check=True):
    todo: ToDo = ActionWrapper.todo

    if clear:
        todo.clear_all()
    tracker_fake = TrackerFake(slots)
    action_to_test().run(CollectingDispatcherFake, tracker_fake, None)

    if check and not check_equals(ActionWrapper.todo, todo_expected):
        print_todo(todo)
        print_todo_dict(todo_expected)
        raise Exception("ToDo does not match the one expected")

if __name__ == "__main__":
    CollectingDispatcherFake.debug = True

    ToDo().store()

    ##########################################################################
    # Test add with wrong datetime 
    # dt = datetime.datetime(year=2020, month=10, day=10, hour=10, minute=10, second=10)
    # test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2020", "time": "10:10:10"}, {"a": [Task("add", dt), ]}, clear=True)

    ##########################################################################
    # Test add
    dt = datetime.datetime(year=2023, month=10, day=10, hour=10, minute=10, second=10)
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2023", "time": "10:10:10"}, {"a": [Task("add", dt), ]}, clear=True)
    test_action(ActionAddAlert, {"category": "a", "tag": "add", "alert": "due ore prima"}, {"a": [Task("add", dt, "due ore prima"), ]}, clear=False)
    test_action(ActionReadTasks, {}, {}, clear=False, check=False)
    test_action(ActionRemoveDeadline, {"category": "a", "tag": "add"}, {"a": [Task("add", None, None), ]}, clear=False)
    test_action(ActionReadTasks, {}, {}, clear=False, check=False)

"""
    # Test remove
    test_action(ActionAddTask, {"category": "b", "tag": "remove", "date": "10/10/2023", "time": "10:10:10"}, {"a": [Task("add", dt), ], "b": [Task("remove", dt), ]}, clear=False)
    test_action(ActionRemoveTask, {"category": "b", "tag": "remove"}, {"a": [Task("add", dt), ], "b": []}, clear=False)

    # Test move to another category
    test_action(ActionMoveTask, {"category": "a", "tag": "add", "category_new": "b"}, {"a": [], "b": [Task("add", dt), ]}, clear=False)
    ##########################################################################
    # Test add
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2023", "time": "10:10:10"}, {"a": [Task("add", dt), ]}, clear=True)
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2023", "time": "10:10:10"}, {"a": [Task("add", dt), ]}, clear=False)

    ##########################################################################
    test_action(ActionCreateUser, {"user": "giggi"}, {}, clear=True)
    test_action(ActionSetUser, {"user": "giggi"}, {}, clear=False)
    # test_action(ActionRemoveUser, {"user": "giggi"}, {}, clear=False)
    test_action(ActionGetUser, {}, {}, clear=False)
    test_action(ActionRenameUser, {"user": "giggi", "user_new": "giggi_v2"}, {}, clear=False)

    ##########################################################################
"""