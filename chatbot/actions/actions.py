import os
from datetime import datetime
from datetime import timedelta
from typing import Any, Sequence, Text, Dict, List, Union
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import AllSlotsReset, SlotSet
from actions.ToDo import ToDo
from actions.ActionsException import ExceptionRasa, ExceptionMissingDeadline, ExceptionMissingAlarm, ExceptionAlertExists
from actions.utils import get_user_new, get_category, get_deadline, get_tag, get_tag_new, get_alert, get_category_new, get_user, get_logical_alert
from actions.Task import Task

class ActionWrapper(Action):
    todo: ToDo = None

    def name(self) -> Text:
        return "None"

class ActionSet(ActionWrapper):
    def name(self) -> Text:
        return "action_reset"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        entity_slot_names = ({"entity": "", "role": "", "slot": ""}, )

        return [SlotSet(x["slot"], tracker.get_latest_entity_values(entity_type=x["entity"], entity_role=x["role"]) if
                                   tracker.get_latest_entity_values(entity_type=x["entity"], entity_role=x["role"]) else []) for x in entity_slot_names]

class ActionReset(ActionWrapper):
    """
    Reset all slots.
    """
    def name(self) -> Text:
        return "action_reset"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        return [AllSlotsReset()]

class ActionCreateUser(ActionWrapper):
    """
    Create a todo for a user.
    
    Slots needed: user.
    """

    def name(self) -> Text:
        return "action_create_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if ToDo.is_user_available(user):
            return []

        ToDo.create_user(user)
        return []

class ActionSetUser(ActionWrapper):
    """
    Set the current loaded user.
    
    Slots needed: user.
    """

    def name(self) -> Text:
        return "action_set_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if not ToDo.is_user_available(user):
            return []

        ActionWrapper.todo = ToDo.load(user)
        dispatcher.utter_message(text=f"Ciao {user}, ho caricato la tua todo-list")
        return []

class ActionRenameUser(ActionWrapper):
    """
    Rename the user to a new name.
    
    Slots needed: user, user_new.
    """

    def name(self) -> Text:
        return "action_rename_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)
        user_new = get_user_new(tracker)

        if not ToDo.is_user_available(user):
            return []

        if ToDo.is_user_available(user_new):
            return []

        ActionWrapper.todo.store(user_new)
        ActionWrapper.todo.remove_user(user)
        ActionWrapper.todo = ToDo.load(user_new)
        dispatcher.utter_message(text=f"Ciao {user_new}, ho modificato e caricato la tua todo-list")
        return []

class ActionGetUser(ActionWrapper):
    """
    Get the user currently loaded.

    No Slots needed.
    """

    def name(self) -> Text:
        return "action_get_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        dispatcher.utter_message(text=f"L'utente attuale è {ToDo.get_loaded_user()}")
        return []

class ActionRemoveUser(ActionWrapper):
    """
    Remove a user from the list of todos.
    
    Slots needed: user.
    """

    def name(self) -> Text:
        return "action_remove_user"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        user = get_user(tracker)

        if not ActionWrapper.todo.is_user_available(user):
            return []

        ActionWrapper.todo.remove_user(user)
        dispatcher.utter_message(text=f"todo-list di {user} cancellata")
        if ActionWrapper.todo.get_loaded_user() == user:
            ActionWrapper.todo = None
        return []

class ActionAddTask(ActionWrapper):
    """
    Add a task to the todo-list and shows the task's informations to the user
    
    Slots needed: tag, category, logical_alert.
    """
    def name(self) -> Text:
        return "action_add_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)
        logical_alert = get_logical_alert(tracker)

        try:
            deadline = get_deadline(tracker)
            
            ActionWrapper.todo.add_task(category, tag, deadline)
            if logical_alert:
                ActionAddAlert().run(dispatcher, tracker, domain)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f'Aggiunta la task con tag "{tag}" nella categoria "{category}"')
        return []

class ActionRemoveTask(ActionWrapper):
    """
    Remove a task from the todo-list and notify that to the user
    
    Slots needed: tag, category.
    """
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

class ActionRemoveDeadline(ActionWrapper):
    """
    Remove a task from the todo-list and notify that to the user
    
    Slots needed: tag, category.
    """
    def name(self) -> Text:
        return "action_remove_deadline"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:
            ActionWrapper.todo.remove_deadline(category, tag)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Deadline ed allarme per attività \"{tag}\" rimossi")
        return [] 

class ActionRemoveCategory(ActionWrapper):
    """
    Remove a task from the todo-list and notify that to the user
    
    Slots needed: category.
    """
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
    """
    If the task is in the todo-list, shows the task and the relative informations to the user
    
    Slots needed: tag, category.
    """
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

        dispatcher.utter_message(text=f"Il task \"{tag}\" ha \"{category}\" come categoria {'' if val.deadline is None else f'e {str(val.deadline)} come scadenza'}{'' if val.alarm is None else f' e {val.alarm} come allarme'}")
        return []

class ActionReadTasks(ActionWrapper):
    """
    If there are some tasks in the todo-list, display a list of tasks to the user
    
    No Slots needed.
    """
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

        n_tasks = 0
        for category in tasks:
            for task in tasks[category]:
                n_tasks+=1
        n_category = len(tasks)
        dispatcher.utter_message(text=f"Hai {n_category} categorie e {n_tasks} task totali")
        return[]

class ActionReadCategories(ActionWrapper):
    """
    If there are some tasks in the todo-list, display list of categories to the user
    
    No Slots needed.
    """
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
    """
    Shows to the user the tasks of a specific category

    Slots needed: category.
    """
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
    """
    Receive old and new tag of a task and replace the old one with the new
    
    Slots needed: tag, tag_new, category.
    """
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
    """
    Modify the category of a task
    
    Slots needed: tag, category, category_new.
    """
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

class ActionMoveModifyTask(ActionWrapper):
    """
    Modify the category of a task
    
    Slots needed: tag, category, tag_new, category_new.
    """
    def name(self) -> Text:
        return "action_move_modify_task"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        tag_new = get_tag_new(tracker)
        category = get_category(tracker)
        category_new = get_category_new(tracker)

        try: 
            ActionWrapper.todo.move_task(category, tag, category_new)
            ActionWrapper.todo.modify_task(category_new, tag, tag_new=tag_new)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Task \"{tag}\" cambiata in \"{ActionWrapper.todo.get_task(category_new, tag_new)}\"")
        return []

class ActionModifyDeadline(ActionWrapper):
    """
    Modify the deadline of a task

    Slots needed: tag, category.
    """
    def name(self) -> Text:
        return "action_modify_deadline"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)
        minutes = 5

        try:
            deadline_new = get_deadline(tracker)
            alert = None
            if ActionWrapper.todo.get_task(category,tag).alarm is not None:
                alert = deadline_new-timedelta(minutes=minutes)
            ActionWrapper.todo.modify_task(category, tag, deadline_new=deadline_new, alarm_new=alert)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Deadline del task \"{tag}\" cambiata in \"{str(deadline_new)}\"")
        return []

class ActionAddAlert(ActionWrapper):
    """
    Add an alert to a task and notify it to the user

    Slots needed: tag, category.
    """
    def name(self) -> Text:
        return "action_add_alert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        minutes = 5
        tag = get_tag(tracker)
        category = get_category(tracker)
        try:
            deadline = ActionWrapper.todo.get_task(category, tag).deadline
            alarm_old = ActionWrapper.todo.get_task(category, tag).alarm

            if alarm_old is not None:
                raise ExceptionAlertExists(category, tag)

            if deadline is None:
                raise ExceptionMissingDeadline()

            alarm = deadline - timedelta(minutes=minutes)

            ActionWrapper.todo.modify_task(category, tag, alarm_new=alarm)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" sarà notificato {str(minutes)} minuti prima della sua deadline")
        return []

class ActionRemoveAlert(ActionWrapper):
    """
    Add an alert to a task and notify it to the user

    Slots needed: tag, category.
    """
    def name(self) -> Text:
        return "action_remove_alert"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        tag = get_tag(tracker)
        category = get_category(tracker)

        try:
            alarm = ActionWrapper.todo.get_task(category, tag).alarm
            if alarm is None:
                raise ExceptionMissingAlarm()
            ActionWrapper.todo.remove_task_alarm(category, tag)
        except ExceptionRasa as e:
            dispatcher.utter_message(text=str(e))
            return []

        dispatcher.utter_message(text=f"Il task \"{tag}\" non verrà più notificato")
        return []

class ActionModifyAlert(ActionWrapper):
    """
    Add an alert to a task and notify it to the user.

    Slots needed: tag, category, alert.
    """
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

        dispatcher.utter_message(text=f"Il task \"{tag}\" sarà ora notificato {str(delta)} della sua deadline")
        return []