class Task:
    def __init__(self, tag, deadline, alarm=None) -> None:
        self.tag = tag
        self.deadline = deadline
        self.alarm = alarm

    @property
    def tag(self):
        return self.tag

    @tag.setter
    def tag(self, value):
        self.tag = value

    @property
    def deadline(self):
        return self.tag

    @deadline.setter
    def deadline(self, value):
        self.deadline = value

    @property
    def alarm(self):
        return self.alarm

    @alarm.setter
    def alarm(self, value):
        self.alarm = value