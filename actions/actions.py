import os
from typing import Any, Sequence, Text, Dict, List
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import AllSlotsReset
from actions.ToDo import ToDo
from actions.ActionsException import ExceptionRasa
from actions.utils import get_user_new, get_category, get_deadline, get_tag, get_tag_new, get_alert, get_category_new, get_user
from actions.Task import Task

class ActionWrapper(Action):
    todo = ToDo.load()

    def name(self) -> Text:
        return "None"

class ActionReset(Action):
    def name(self) -> Text:
        return "action_reset"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [AllSlotsReset()]

class ActionCreateUser(Action):
    def name(self) -> Text:
        return "action_create_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if ToDo.is_user_available(user):
            dispatcher.utter_message(text=f"Todo-list di {user} già esistente")
            return []

        ToDo.create_user(user)
        dispatcher.utter_message(text=f"Creata una todo-list per {user}")
        return []

class ActionSetUser(Action):
    def name(self) -> Text:
        return "action_set_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if not ToDo.is_user_available(user):
            dispatcher.utter_message(text=f"Todo-list di {user} non esistente")
            return []

        ActionWrapper.todo = ToDo.load(user)
        dispatcher.utter_message(text=f"Caricata la todo-list di {user}")
        return []

class ActionRenameUser(Action):
    def name(self) -> Text:
        return "action_rename_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)
        user_new = get_user_new(tracker)

        if not ToDo.is_user_available(user):
            dispatcher.utter_message(text=f"Todo-list di {user} non esistente")
            return []

        if ToDo.is_user_available(user_new):
            dispatcher.utter_message(text=f"Todo-list di {user_new} già esistente")
            return []

        ActionWrapper.todo.store(user_new)
        ActionWrapper.todo.remove_user(user)
        ActionWrapper.todo = ToDo.load(user_new)
        dispatcher.utter_message(text=f"Modificata e caricata la todo-list di {user_new}")
        return []

class ActionGetUser(Action):
    def name(self) -> Text:
        return "action_get_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(text=f"L'utente attuale è {ToDo.get_loaded_user()}")
        return []

class ActionRemoveUser(Action):
    def name(self) -> Text:
        return "action_remove_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if not ActionWrapper.todo.is_user_available(user):
            dispatcher.utter_message(text=f"Todo-list di {user} non esistente")
            return []

        if ActionWrapper.todo.get_loaded_user() == user:
            ActionWrapper.todo = ToDo.load()
            dispatcher.utter_message(text=f"Caricata la todo-list di default")

        ActionWrapper.todo.remove_user(user)
        dispatcher.utter_message(text=f"Cancellata la todo-list di {user}")
        return []

class ActionAddTask(ActionWrapper):
    """Add a task to the todo-list and shows the task's informations to the user """
    def name(self) -> Text:
        return "action_add_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:            
            deadline = get_deadline(tracker)
            
            ActionWrapper.todo.add_task(category, tag, deadline)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Aggiunto \"{tag}\" con \"{category}\" come categoria e \"{str(deadline)}\" come scadenza")
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
        return "action_remove_category"

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

        dispatcher.utter_message(text=f"Il task \"{tag}\" ha \"{category}\" come categoria e \"{str(val.deadline)}\" come scadenza{'' if val.alarm is None else f' e {val.alarm} come allarme'}")
        return []

class ActionReadTasks(ActionWrapper):
    """If there are some tasks in the todo-list, display a list of tasks to the user"""
    def name(self) -> Text:
        return "action_read_tasks"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        try:
            tasks: Dict[str, Sequence[Task]] = ActionWrapper.todo.get_tasks()
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        for category in tasks:
            message = f"Per la categoria \"{category}\" sono presenti i seguenti task:"
            for task in tasks[category]:
                message += f"\n - {task}"
            dispatcher.utter_message(text=(message+"\n"))
        return[]

class ActionReadCategories(ActionWrapper):
    """If there are some tasks in the todo-list, display list of categories to the user"""
    def name(self) -> Text:
        return "action_read_categories"

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

class ActionMoveTask(ActionWrapper):
    """Modify the category of a task"""
    def name(self) -> Text:
        return "action_move_task"

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

        try: 
            deadline_new = get_deadline(tracker)
            
            ActionWrapper.todo.modify_task(category, tag, deadline_new=deadline_new)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Deadline del task \"{tag}\" cambiata in \"{str(deadline_new)}\"")
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
        category = get_category(tracker)
        delta = get_alert(tracker)

        try:
            ActionWrapper.todo.modify_task(category, tag, alarm_new=delta)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        # ActionWrapper.todo[tag]["alert"].append(datetime.timedelta(delta.to_datetime()).replace(microsecond=0))
        # dispatcher.utter_message(text=f"Il task \"{tag}\" sarà notificato {datetime.timedelta(delta.to_datetime()).replace(microsecond=0)} prima della sua deadline")
        dispatcher.utter_message(text=f"Il task \"{tag}\" sarà notificato {str(delta)} prima della sua deadline")
        return []

class ActionRemoveAlert(ActionWrapper):
    """Add an alert to a task and notify it to the user"""
    def name(self) -> Text:
        return "action_remove_alert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:
            ActionWrapper.todo.remove_task_alarm(category, tag)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" non verrà più notificato")
        return []

class ActionModifyAlert(ActionWrapper):
    """Add an alert to a task and notify it to the user"""
    def name(self) -> Text:
        return "action_modify_alert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)
        delta = get_alert(tracker)

        try:
            ActionWrapper.todo.modify_task(category, tag, alarm_new=delta)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" sarà ora notificato {str(delta)} prima della sua deadline")
        return []