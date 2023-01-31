from dateutil import parser
import json
import datetime

class Task:
    def __init__(self, tag, deadline: datetime.datetime=None, alarm: datetime.datetime=None) -> None:
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
        deadline_refactor = '' if self.deadline is None else self.deadline.strftime('%m/%d/%Y %H:%M:%S')
        alarm_refactor = '' if self.alarm is None else  self.alarm.strftime('%m/%d/%Y %H:%M:%S')
        deadline_string = f" con {deadline_refactor} come scadenza" if deadline_refactor!='' else ''
        alarm_string = f" e {alarm_refactor} come allarme" if alarm_refactor!='' else ''
        return f"Task \"{self.tag}\"{deadline_string}{alarm_string}" 