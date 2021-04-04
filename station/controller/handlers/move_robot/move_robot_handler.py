import json
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from handlers.handler import Handler


class MoveRobotHandler(Handler):
    def handle(self, handled_data=None):
        pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        pub.publish(handled_data['goal'])

        self.is_finished = False
        rospy.Subscriber('movement_vectors_string', String, self.is_vector_at_destination)

        while not self.is_finished:
            pass

        return handled_data, True

    def is_vector_at_destination(self, vector_json):
        vector = json.loads(str(vector_json.data))
        self.is_finished = vector == (0, 0, 0)
