import rospy
from std_msgs.msg import Bool
from handlers.handler import Handler


class WaitForFrontendCycleStartHandler(Handler):
    is_finished = False
    start_cycle_subscriber = None

    def handle_start_cycle(self, start):
        print('Finished : wait for frontend cycle start handler')  # TODO : Remove print
        self.is_finished = start

    def handle(self, handled_data=None):
        print('Looping in wait for frontend cycle start handler')  # TODO : Remove print

        if not self.start_cycle_subscriber:
            self.start_cycle_subscriber = rospy.Subscriber("start_cycle", Bool, self.handle_start_cycle)

        return handled_data, self.is_finished

    def unregister(self):
        self.start_cycle_subscriber.unregister()
