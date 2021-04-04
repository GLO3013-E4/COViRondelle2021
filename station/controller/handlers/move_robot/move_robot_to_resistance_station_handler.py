import json
import rospy

from std_msgs.msg import String
from handlers.handler import Handler
from handlers.move_robot.move_robot_handler import MoveRobotHandler
from hard_coded_positons import RESISTANCE_STATION_POSITION


class MoveRobotToResistanceStationHandler(Handler):
    def handle(self, handled_data=None):
        # TODO : Implement MoveRobotToResistanceStationHandler
        return handled_data, True


class MoveRobotToCommandPanelHandler(Handler):
    def handle(self, handled_data=None):
        move_robot_handler = MoveRobotHandler()

        handled_data['goal'] = RESISTANCE_STATION_POSITION
        handled_data['destination'] = 'resistance_station'

        is_finished = False
        while not is_finished:
            handled_data, is_finished = move_robot_handler.handle(handled_data)

        # TODO: avancer un peu pour coller la station
        pub = rospy.Publisher('movement_vectors_string', String, queue_size=1)
        pub.publish(json.dumps("(5, 0, 1)"))

        return handled_data, True
