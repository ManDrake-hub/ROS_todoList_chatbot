from dateutil import parser
import json

class Task:
    def __init__(self, tag, deadline=None, alarm=None) -> None:
        self.tag = tag
        self.deadline = deadline
        self.alarm = alarm

    def remove_deadline(self):
        self.deadline = None

    def remove_alarm(self):
        self.alarm = None

    def encode(self) -> str:
        return json.dumps({"tag": self.tag, "deadline": str(self.deadline) if self.deadline is not None else None, "alarm": str(self.alarm) if self.alarm is not None else None})

    @staticmethod
    def decode(encoded: str):
        d = json.loads(encoded)
        return Task(d["tag"], parser.parse(d["deadline"], dayfirst=True) if d["deadline"] is not None else None, parser.parse(d["alarm"], dayfirst=True) if d["alarm"] is not None else None)

    def __str__(self) -> str:
        return f"Task \"{self.tag}\" {'' if self.deadline is None else f'con {str(self.deadline)} come scadenza'}{'' if self.alarm is None else f' e {self.alarm} come allarme'}"