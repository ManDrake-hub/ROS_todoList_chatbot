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

##########################################################################
# Category exceptions                                                    #
##########################################################################
class ExceptionMissingCategory(ExceptionRasa):
    def __init__(self, category) -> None:
        super().__init__(f"Categoria \"{category}\" non esistente")

class ExceptionNoCategories(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Nessuna categoria trovata")

##########################################################################
# Date and time Exceptions                                               #
##########################################################################
class ExceptionDateFormatInvalid(ExceptionRasa):
    pass # <-- Add feedback like other exceptions

class ExceptionTimeFormatInvalid(ExceptionRasa):
    pass # <-- Add feedback like other exceptions