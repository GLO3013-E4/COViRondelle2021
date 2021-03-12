import rospy
from std_msgs.msg import Bool
from handlers.handler import Handler


# TODO : Test this handler (how do we mock rospy?)
class WaitForFrontendCycleStartHandler(Handler):
    def handle(self, handled_data=None):
        print('Looping in wait for frontend cycle start handler')  # TODO : Remove print
        is_finished = rospy.wait_for_message("start", Bool)

        if is_finished:
            print('Done!')  # TODO : Remove print

        return handled_data, is_finished
