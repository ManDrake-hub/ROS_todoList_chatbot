class Task:
    def __init__(self, tag, deadline, alarm=None) -> None:
        self.tag = tag
        self.deadline = deadline
        self.alarm = alarm