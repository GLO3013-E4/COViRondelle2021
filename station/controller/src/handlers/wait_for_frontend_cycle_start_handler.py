import rospy
from std_msgs.msg import Bool
from controller.src.handlers.handler import Handler


# TODO : Test this handler (how do we mock rospy?)
class WaitForFrontendCycleStartHandler(Handler):
    is_finished = False

    def handle_start(self, start: Bool):
        self.is_finished = start

    def handle(self, handled_data=None):
        self.start_subscriber = rospy.Subscriber("start", Bool, self.handle_start)

        return handled_data, self.is_finished

    def unregister(self):
        self.start_subscriber.unregister()
