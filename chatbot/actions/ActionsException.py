##########################################################################
# Father exception                                                       #
##########################################################################
class ExceptionRasa(Exception):
    """Class used to catch only custom exception in one line"""
    pass

##########################################################################
# Task exceptions                                                        #
##########################################################################
class ExceptionMissingTask(ExceptionRasa):
    def __init__(self, category, tag) -> None:
        super().__init__(f"Task \"{tag}\" nella categoria \"{category}\" non trovato")

class ExceptionNoTasks(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Nessuna task trovata")

class ExceptionTaskExists(ExceptionRasa):
    def __init__(self, category, tag) -> None:
        super().__init__(f"Task \"{tag}\" nella categoria \"{category}\" già esistente")

class ExceptionMissingDeadline(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Deadline non specificata")

class ExceptionMissingAlarm(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Alarm non attivo")

class ExceptionAlertExists(ExceptionRasa):
    def __init__(self, category, tag) -> None:
        super().__init__(f"Alert per la task \"{tag}\" nella categoria \"{category}\" già esistente")

##########################################################################
# Category exceptions                                                    #
##########################################################################
class ExceptionMissingCategory(ExceptionRasa):
    def __init__(self, category) -> None:
        super().__init__(f"Categoria \"{category}\" non esistente")

class ExceptionNoCategories(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"ToDo-list vuota")

##########################################################################
# Date and time Exceptions                                               #
##########################################################################
class ExceptionDateTimeFormatInvalid(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Deadline in un formato non riconosciuto")

class ExceptionDateTimeBeforeNow(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Deadline antecedente al tempo corrente")