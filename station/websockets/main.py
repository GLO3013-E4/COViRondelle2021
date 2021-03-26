#!/usr/bin/env python3
import base64
import json
import threading
from io import BytesIO
import rospy
from PIL import Image
from flask import Flask
from flask_socketio import SocketIO
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path

app = Flask(__name__)
socket = SocketIO(app, cors_allowed_origins="*")


def to_json(data):
    return json.dumps(data)


def to_base64(data):
    image = Image.fromarray(data)
    buffered = BytesIO()
    image.save(buffered, format="JPEG")
    return base64.b64encode(buffered.getvalue())


def handle_robot_consumption(robot_consumption):
    socket.emit("robot_consumption", robot_consumption.data)


def handle_world_camera_image_raw(image):
    img_base64 = to_base64(image.data)
    json_data = to_json({"tableImage": img_base64})
    socket.emit("table_image", json_data)


def handle_robot(pose):
    json_data = to_json({"realTrajectoryCoordinate": {"x": pose.position.x, "y": pose.position.y}})
    socket.emit("real_trajectory_coordinate", json_data)


def handle_path(path):
    poses = []

    for pose in path.poses:
        poses.append({"x": pose.position.x, "y": pose.position.y})

    json_data = to_json({"plannedTrajectoryCoordinates": poses})
    socket.emit("planned_trajectory_coordinates", json_data)


def handle_grip(grip):
    json_data = to_json({"puckInGrip": grip.data})
    socket.emit("grip_state", json_data)


def handle_resistance(resistance):
    json_data = to_json({"resistance": int(resistance.data)})
    socket.emit("resistance", json_data)


def handle_puck_colors(puck_colors):
    json_data = to_json({"puckColors": puck_colors.data.split(",")})
    socket.emit("puck_colors", json_data)


def handle_puck_corners(puck_corners):
    json_data = to_json({"puckFirstCorner": puck_corners.data.split(",")[0]})
    socket.emit("puck_first_corner", json_data)


def handle_end(_):
    json_data = to_json({"currentStep": "CycleEndedAndRedLedOn"})
    socket.emit("current_step", json_data)


def websockets():
    start_cycle_publisher = rospy.Publisher("start_cycle", Bool, queue_size=10)
    rospy.init_node("websockets", anonymous=True)
    rate = rospy.Rate(1)

    rospy.Subscriber("robot_consumption", String, handle_robot_consumption)
    rospy.Subscriber("world_camera/image_raw", ROSImage, handle_world_camera_image_raw)
    rospy.Subscriber("robot", Pose, handle_robot)
    rospy.Subscriber("path", Path, handle_path)
    rospy.Subscriber("grip", Bool, handle_grip)
    rospy.Subscriber("resistance", Float32, handle_resistance)
    rospy.Subscriber("puck_colors", String, handle_puck_colors)
    rospy.Subscriber("puck_corners", String, handle_puck_corners)
    rospy.Subscriber("end", Bool, handle_end)  # TODO : Remove end, subscribe to current_step

    @socket.on('start_cycle')
    def handle_start_cycle():
        start_cycle_publisher.publish(True)

    is_running = False

    while not rospy.is_shutdown():
        if not is_running:
            is_running = True
            threading.Thread(target=lambda: socket.run(app, host='0.0.0.0', port=4000)).start()

        rate.sleep()


if __name__ == '__main__':
    try:
        websockets()
    except rospy.ROSInterruptException:
        pass
