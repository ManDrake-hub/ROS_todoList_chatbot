from actions import ActionAddTask
from utils import CollectingDispatcherFake, TrackerFake

if __name__ == "__main__":
    tracker_fake = TrackerFake()
    dispatcher_fake = CollectingDispatcherFake

    ActionAddTask().run(dispatcher_fake, tracker_fake, {})