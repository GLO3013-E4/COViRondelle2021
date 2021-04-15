import rospy
from std_msgs.msg import Bool
from handlers.handler import Handler


class WaitForFrontendCycleStartHandler(Handler):
    def __init__(self):
        self.rate = rospy.Rate(1)

    def initialize(self):
        self.sub = rospy.Subscriber("start_cycle", Bool, self.handle_start_cycle)
        self.is_finished = False

    def handle(self, handled_data):
        self.initialize()

        while not self.is_finished:
            rospy.logerr("waiting for frontend start cycle")
            self.rate.sleep()

        return handled_data

    def handle_start_cycle(self, _):
        self.is_finished = True

    def unregister(self):
        self.sub.unregister()
