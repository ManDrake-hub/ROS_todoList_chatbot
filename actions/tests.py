from actions import ActionAddTask
from utils import CollectingDispatcherFake, TrackerFake

if __name__ == "__main__":
    tracker_fake = TrackerFake()
    dispatcher_fake = CollectingDispatcherFake

    tracker_fake.slots["tag"] = "giggi"
    tracker_fake.slots["category"] = "spesa"
    tracker_fake.slots["date"] = "12/12/12"
    tracker_fake.slots["time"] = "10:10:10"

    ActionAddTask().run(dispatcher_fake, tracker_fake, {})