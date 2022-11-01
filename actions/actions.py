from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from ToDo import ToDo
from ActionsException import ExceptionRasa
from utils import get_category, get_deadline, get_info, get_tag, sequence_to_str, get_tag_new, get_alert, get_category_new

import datetime

class ActionWrapper(Action):
    todo = ToDo.load()

class ActionAddTask(ActionWrapper):
    """Add a task to the todo-list and shows the task's informations to the user """
    def name(self) -> Text:
        return "action_add_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag, category, deadline = get_info(tracker)

        try:        
            ActionWrapper.todo.add_task(category, tag, deadline)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Aggiunto \"{tag}\" con \"{category}\" come categoria e \"{deadline}\" come scadenza")
        return []

class ActionRemoveTask(ActionWrapper):
    """Remove a task from the todo-list and notify that to the user"""
    def name(self) -> Text:
        return "action_remove_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:
            ActionWrapper.todo.remove_task(category = category,tag=tag)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Task \"{tag}\" rimosso")
        return []

class ActionRemoveCategory(ActionWrapper):
    """Remove a task from the todo-list and notify that to the user"""
    def name(self) -> Text:
        return "action_remove_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        category = get_category(tracker)

        try:
            ActionWrapper.todo.remove_category(category)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Categoria \"{category}\" rimossa")
        return []

class ActionReadTask(ActionWrapper):
    """If the task is in the todo-list, shows the task and the relative informations to the user"""
    def name(self) -> Text:
        return "action_read_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:
            val = ActionWrapper.todo.get_task(category, tag)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" ha \"{category}\" come categoria e \"{val.deadline}\" come scadenza")
        return []

class ActionReadTasks(ActionWrapper):
    """If there are some tasks in the todo-list, display a list of tasks to the user"""
    def name(self) -> Text:
        return "action_read_tasks"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        try:
            categories = ActionWrapper.todo.get_tasks()
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        for category in categories:
            message = f"For the category \"{category}\" you have the following tasks:"
            for tag in category:
                message += f"\n - {tag}"
            dispatcher.utter_message(text=(message+"\n"))
        return[]

class ActionReadCategories(ActionWrapper):
    """If there are some tasks in the todo-list, display list of categories to the user"""
    def name(self) -> Text:
        return "action_read_tasks"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        try:
            categories = ActionWrapper.todo.get_categories()
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []
        
        message = f"Sono presenti le seguenti categorie:"
        for category in categories:
            message += f"\n - {category}"

        dispatcher.utter_message(text=message)
        return []

class ActionReadCategory(ActionWrapper):
    """Shows to the user the tasks of a specific category"""
    def name(self) -> Text:
        return "action_read_category"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        category = get_category(tracker)

        try:
            tags = ActionWrapper.todo.get_tasks_of_category(category)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []
        
        message = f"La categoria ha i seguenti task associati:"
        for tag in tags:
            message += f"\n - {tag}"

        dispatcher.utter_message(text=message)
        return []

class ActionModifyTaskName(ActionWrapper):
    """Receive old and new tag of a task and replace the old one with the new"""
    def name(self) -> Text:
        return "action_modify_task_name"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        tag_new = get_tag_new(tracker)
        category = get_category(tracker)

        try: 
            ActionWrapper.todo.modify_task(category, tag, tag_new)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" è stato rinominato come: \"{tag_new}\"")
        return []

class ActionModifyCategory(ActionWrapper):
    """Modify the category of a task"""
    def name(self) -> Text:
        return "action_modify_categories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)
        category_new = get_category_new(tracker)

        try: 
            ActionWrapper.todo.move_task(category, tag, category_new)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Categoria del task \"{tag}\" cambiata in \"{category_new}\"")
        return []

class ActionModifyDeadline(ActionWrapper):
    """Modify the deadline of a task"""
    def name(self) -> Text:
        return "action_modify_deadline"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)
        deadline_new = get_deadline(tracker)

        try: 
            ActionWrapper.todo.modify_task(category, tag, deadline_new=deadline_new)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Deadline del task \"{tag}\" cambiata in \"{deadline_new}\"")
        return []

# TO-DO specify the type of vars cause this function not works
# Replace the ActionWrapper calls to merge that with the changes
# done to the class
class ActionAddAlert(ActionWrapper):
    """Add an alert to a task and notify it to the user"""
    def name(self) -> Text:
        return "action_add_alert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        delta = get_alert(tracker)

        if not tag in ActionWrapper.todo:
            dispatcher.utter_message(text=f"Task \"{tag}\" non in elenco")
            return []

        ActionWrapper.todo[tag]["alert"].append(datetime.timedelta(delta.to_datetime()).replace(microsecond=0))
        dispatcher.utter_message(text=f"Il task \"{tag}\" sarà notificato {datetime.timedelta(delta.to_datetime()).replace(microsecond=0)} prima della sua deadline")
        return []
