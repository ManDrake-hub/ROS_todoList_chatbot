class ExceptionRasa(Exception):
    pass

class ExceptionMissingCategory(ExceptionRasa):
    def __init__(self, category) -> None:
        super().__init__(f"Categoria \"{category}\" non esistente")

class ExceptionMissingTask(ExceptionRasa):
    def __init__(self, category, tag) -> None:
        super().__init__(f"Task \"{tag}\" nella categoria \"{category}\" non trovato")

class ExceptionNoCategories(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Nessuna categoria trovata")

class ExceptionNoTasks(ExceptionRasa):
    def __init__(self) -> None:
        super().__init__(f"Nessuna task trovata")

class ExceptionTaskExists(ExceptionRasa):
    def __init__(self, category, tag) -> None:
        super().__init__(f"Task \"{tag}\" nella categoria \"{category}\" già esistente")