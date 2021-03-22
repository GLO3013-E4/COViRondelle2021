import rospy
from std_msgs.msg import String
from handlers.handler import Handler


class WaitForRobotReadyStateHandler(Handler):
    is_finished = False
    ready_subscriber = None

    def handle_robot_consumption(self, _):
        print('Finished : wait for robot ready state handler')  # TODO : Remove print
        self.is_finished = True

    def handle(self, handled_data=None):
        print('Looping in wait for robot ready state handler')  # TODO : Remove print

        if not self.ready_subscriber:
            self.ready_subscriber = rospy.Subscriber("robot_consumption", String, self.handle_robot_consumption)

        return handled_data, self.is_finished

    def unregister(self):
        self.ready_subscriber.unregister()
