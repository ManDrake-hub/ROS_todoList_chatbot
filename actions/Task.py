class Task:
    def __init__(self, tag, deadline=None, alarm=None) -> None:
        self.tag = tag
        self.deadline = deadline
        self.alarm = alarm

    def remove_deadline(self):
        self.deadline = None

    def remove_alarm(self):
        self.alarm = None

    def __str__(self) -> str:
        return f"Task \"{self.tag}\" {'' if self.deadline is None else f'con {str(self.deadline)} come scadenza'}{'' if self.alarm is None else f' e {self.alarm} come allarme'}"