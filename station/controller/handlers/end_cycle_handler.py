from handlers.handler import Handler
import rospy


class EndCycleHandler(Handler):
    def handle(self, handled_data):
        handled_data["red_light_pub"].publish(True)
        rospy.sleep(10)
        return handled_data

    def unregister(self):
        pass
