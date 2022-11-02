from typing import Dict, List, Any
from actions.actions import ActionWrapper, ActionAddTask, ActionRemoveTask, ActionMoveTask, ActionCreateUser, ActionSetUser, ActionRemoveUser, ActionGetUser
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

def test_action(action_to_test: ActionWrapper, slots: Dict[str, Any], todo_expected: Dict[str, List[Task]], clear: bool=True):
    todo: ToDo = ActionWrapper.todo

    if clear:
        todo.clear_all()
    tracker_fake = TrackerFake(slots)
    action_to_test().run(CollectingDispatcherFake, tracker_fake, None)

    if not check_equals(ActionWrapper.todo, todo_expected):
        print_todo(todo)
        print_todo_dict(todo_expected)
        raise Exception("ToDo does not match the one expected")

if __name__ == "__main__":
    CollectingDispatcherFake.debug = True

    ##########################################################################
    # Test add
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2020", "time": "10:10:10"}, {"a": [Task("add", "10/10/2020 10:10:10"), ]}, clear=True)

    # Test remove
    test_action(ActionAddTask, {"category": "b", "tag": "remove", "date": "10/10/2020", "time": "10:10:10"}, {"a": [Task("add", "10/10/2020 10:10:10"), ], "b": [Task("remove", "10/10/2020 10:10:10"), ]}, clear=False)
    test_action(ActionRemoveTask, {"category": "b", "tag": "remove"}, {"a": [Task("add", "10/10/2020 10:10:10"), ], "b": []}, clear=False)

    # Test move to another category
    test_action(ActionMoveTask, {"category": "a", "tag": "add", "category_new": "b"}, {"a": [], "b": [Task("add", "10/10/2020 10:10:10"), ]}, clear=False)
    ##########################################################################
    # Test add
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2020", "time": "10:10:10"}, {"a": [Task("add", "10/10/2020 10:10:10"), ]}, clear=True)
    test_action(ActionAddTask, {"category": "a", "tag": "add", "date": "10/10/2020", "time": "10:10:10"}, {"a": [Task("add", "10/10/2020 10:10:10"), ]}, clear=False)

    ##########################################################################
    test_action(ActionCreateUser, {"user": "giggi"}, {}, clear=True)
    test_action(ActionSetUser, {"user": "giggi"}, {}, clear=False)
    # test_action(ActionRemoveUser, {"user": "giggi"}, {}, clear=False)
    test_action(ActionGetUser, {}, {}, clear=False)
