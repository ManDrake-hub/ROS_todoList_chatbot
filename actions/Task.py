class Task:
    def __init__(self, tag, deadline, alarm=None) -> None:
        self.tag = tag
        self.deadline = deadline
        self.alarm = alarm

    def __str__(self) -> str:
        return f"Task \"{self.tag}\" con \"{str(self.deadline)}\" come scadenza{'' if self.alarm is None else f' e {self.alarm} come allarme'}"