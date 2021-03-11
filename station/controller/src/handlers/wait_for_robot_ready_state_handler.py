import rospy
from std_msgs.msg import Bool
from controller.src.handlers.handler import Handler


# TODO : Test this handler (how do we mock rospy?)
class WaitForRobotReadyStateHandler(Handler):
    is_finished = False

    def handle_ready(self, ready: Bool):
        self.is_finished = ready

    def handle(self, handled_data=None):
        self.ready_subscriber = rospy.Subscriber("ready", Bool, self.handle_ready)

        return handled_data, self.is_finished

    def unregister(self):
        self.ready_subscriber.unregister()
