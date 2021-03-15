import rospy
from std_msgs.msg import Bool
from handlers.handler import Handler


class WaitForRobotReadyStateHandler(Handler):
    is_finished = False
    ready_subscriber = None

    def handle_ready(self, ready):
        print('Finished : wait for robot ready state handler')  # TODO : Remove print
        self.is_finished = ready

    def handle(self, handled_data=None):
        print('Looping in wait for robot ready state handler')  # TODO : Remove print

        if not self.ready_subscriber:
            self.ready_subscriber = rospy.Subscriber("ready", Bool, self.handle_ready)

        return handled_data, self.is_finished

    def unregister(self):
        self.ready_subscriber.unregister()
