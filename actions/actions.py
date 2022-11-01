from typing import Any, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from ToDo import ToDo
from utils import get_categories, get_deadline, get_info, get_tag, sequence_to_str, get_tag_new, get_alert

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
        if not ActionWrapper.todo.add_task(category= category, tag=tag , deadline=deadline):
            dispatcher.utter_attachment(text=f"Task {tag} già esistente, nessuna modifica è stata effettuata")
            return []

        dispatcher.utter_message(text=f"Aggiunto {tag} con {sequence_to_str(category)} come categorie e {deadline} come scadenza")
        return []

class ActionRemoveTask(ActionWrapper):
    """Remove a task from the todo-list and notify that to the user"""
    def name(self) -> Text:
        return "action_remove_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        ActionWrapper.todo.remove_task("")

        dispatcher.utter_message(text=f"{tag} rimosso")
        return []

class ActionRemoveCategory(ActionWrapper):
    """Remove a task from the todo-list and notify that to the user"""
    def name(self) -> Text:
        return "action_remove_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        ActionWrapper.todo.pop(tag, None)

        dispatcher.utter_message(text=f"{tag} rimosso")
        return []

class ActionReadTask(ActionWrapper):
    """If the task is in the todo-list, shows the task and the relative informations to the user"""
    def name(self) -> Text:
        return "action_read_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        val = ActionWrapper.todo.get(tag, None)

        if val is None:
            dispatcher.utter_message(text=f"{tag} non trovato")
            return []

        categories, deadline = val["categories"], val["deadline"]
        dispatcher.utter_message(text=f"{tag} ha {sequence_to_str(categories)} come categorie e {deadline} come scadenza")
        return []

class ActionReadTasks(ActionWrapper):
    """If there are some tasks in the todo-list, shows list of tasks to the user"""
    def name(self) -> Text:
        return "action_read_tasks"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tags = ActionWrapper.todo.keys()

        if not tags:
            dispatcher.utter_message(text=f"No tags available")
            return []
        
        message = f"You have the following tasks:"
        for tag in tags:
            message += f"\n - {tag}"

        dispatcher.utter_message(text=message)
        return []

class ActionReadCategories(ActionWrapper):
    """If there are some tasks in the todo-list, shows list of tasks to the user"""
    def name(self) -> Text:
        return "action_read_tasks"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        vals = ActionWrapper.todo.values()
        categories = set([y for y in [x["categories"] for x in vals]])

        if not categories:
            dispatcher.utter_message(text=f"No categories available")
            return []
        
        message = f"You have the following categories:"
        for category in categories:
            message += f"\n - {category}"

        dispatcher.utter_message(text=message)
        return []

class ActionReadCategory(ActionWrapper):
    """Shows to the user the tasks that have a specific category"""
    def name(self) -> Text:
        return "action_read_category"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        category = get_categories(tracker)

        tags = [x for x in ActionWrapper.todo.keys() if category in ActionWrapper.todo[x]["categories"]]

        if not tags:
            dispatcher.utter_message(text=f"Nessun task associato a questa categoria")
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

        if not tag in ActionWrapper.todo:
            dispatcher.utter_message(text=f"Nessuna task in elenco")
            return []

        val = ActionWrapper.todo[tag]
        ActionWrapper.todo.pop(tag)
        ActionWrapper.todo[tag_new] = val

        dispatcher.utter_message(text=f"Rinominato il task {tag} a {tag_new}")
        return []

class ActionModifyCategories(ActionWrapper):
    """Modify the categories of a task"""
    def name(self) -> Text:
        return "action_modify_categories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        categories_new = get_categories(tracker)

        if not tag in ActionWrapper.todo:
            dispatcher.utter_message(text=f"Nessuna task '{tag}' in elenco")
            return []

        ActionWrapper.todo[tag]["categories"] = categories_new

        dispatcher.utter_message(text=f"Categorie del task {tag} cambiate a {sequence_to_str(categories_new)}")
        return []

class ActionModifyDeadline(ActionWrapper):
    """Modify the deadline of a task"""
    def name(self) -> Text:
        return "action_modify_deadline"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        deadline_new = get_deadline(tracker)

        if not tag in ActionWrapper.todo:
            dispatcher.utter_message(text=f"Nessuna task '{tag}' in elenco")
            return []

        ActionWrapper.todo[tag]["dealine"] = deadline_new

        dispatcher.utter_message(text=f"Categorie del task {tag} cambiate a {deadline_new}")
        return []

# TO-DO specify the type of vars cause this function not works
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
            dispatcher.utter_message(text=f"Task '{tag}' non in elenco")
            return []

        ActionWrapper.todo[tag]["alert"].append(datetime.timedelta(delta.to_datetime()).replace(microsecond=0))
        dispatcher.utter_message(text=f"Il task {tag} sarà notificato {datetime.timedelta(delta.to_datetime()).replace(microsecond=0)} prima della sua deadline")
        return []

