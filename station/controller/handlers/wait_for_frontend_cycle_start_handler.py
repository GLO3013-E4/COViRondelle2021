import rospy
from std_msgs.msg import Bool
from handlers.handler import Handler


class WaitForFrontendCycleStartHandler(Handler):
    def initialize(self):
        self.sub = rospy.Subscriber("start_cycle", Bool, self.handle_start_cycle)
        self.is_finished = False

    def handle(self, handled_data):
        self.initialize()

        if not self.is_finished:
            pass

        return handled_data

    def handle_start_cycle(self, _):
        self.is_finished = True

    def unregister(self):
        self.sub.unregister()
