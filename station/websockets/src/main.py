#!/usr/bin/env python
import rospy
import json
from flask import Flask
from flask_socketio import SocketIO
from std_msgs.msg import Bool, Float, StringArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

app = Flask(__name__)
socket = SocketIO(app, cors_allowed_origins="*")


def to_json(data):
    return json.dumps(data).encode("utf-8")


def handle_ready(_):
    # TODO : Make sure this works once cycle_ready is implemented
    socket.emit("cycle_ready")


def handle_world_camera_image_raw(image):
    # TODO : Make sure this works once world_camera/image_row is implemented (most likely not this way)
    json_data = to_json({"tableImage": image})
    socket.emit("table_image", json_data)


def handle_robot(coordinate):
    # TODO : Make sure this works once robot (coordinate) is implemented (most likely not this way)
    json_data = to_json({"realTrajectoryCoordinate": {"x": coordinate.x, "y": coordinate.y}})
    socket.emit("real_trajectory_coordinate", json_data)


def handle_path(path):
    # TODO : Make sure this works once path is implemented (most likely not this way)
    coordinates = []

    for coordinate in path.coordinates:
        coordinates.append({"x": coordinate.x, "y": coordinate.y})

    json_data = to_json({"plannedTrajectoryCoordinates": coordinates})
    socket.emit("planned_trajectory_coordinates", json_data)


def handle_resistance(resistance):
    # TODO : Make sure this works once resistance is implemented
    json_data = to_json({"resistance": resistance})
    socket.emit("resistance", json_data)


def handle_puck_colors(puck_colors):
    # TODO : Make sure this works once puck_colors is implemented
    json_data = to_json({"puckColors": puck_colors})
    socket.emit("puck_colors", json_data)


def handle_puck_corners(puck_corners):
    # TODO : Make sure this works once puck_corners is implemented
    json_data = to_json({"firstPuckCorner": puck_corners[0]})
    socket.emit("first_puck_corner", json_data)


def handle_end(_):
    # TODO : Make sure this works once end is implemented
    json_data = to_json({"currentStep": "CycleEndedAndRedLedOn"})
    socket.emit("current_step", json_data)


def websockets():
    # pub = rospy.Publisher("chatter", String, queue_size=10)
    rospy.init_node("websockets", anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber("ready", Bool, handle_ready)
    rospy.Subscriber("world_camera/image_raw", Image, handle_world_camera_image_raw)
    rospy.Subscriber("robot", Pose, handle_robot)
    rospy.Subscriber("path", Path, handle_path)
    rospy.Subscriber("resistance", Float, handle_resistance)
    rospy.Subscriber("puck_colors", StringArray, handle_puck_colors)
    rospy.Subscriber("puck_corners", StringArray, handle_puck_corners)
    rospy.Subscriber("end", Bool, handle_end)

    socket.run(app)

    while not rospy.is_shutdown():
        # TODO : Handle start_cycle

        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)

        rate.sleep()


if __name__ == '__main__':
    try:
        websockets()
    except rospy.ROSInterruptException:
        pass
