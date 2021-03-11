import rospy
from controller.src.handlers.handler import Handler
from std_msgs.msg import Bool


class WaitForFrontendCycleStartHandler(Handler):
    is_finished = False

    def handle_start(self, start: Bool):
        self.is_finished = start

    def handle(self, handled_data=None):
        rospy.Subscriber("start", Bool, self.handle_start)

        return handled_data, self.is_finished
