#!/usr/bin/env python
import rospy
import json
from flask import Flask
from flask_socketio import SocketIO
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

app = Flask(__name__)
socket = SocketIO(app, cors_allowed_origins="*")


def to_json(data):
    return json.dumps(data).encode('utf-8')


def handle_ready(_):
    socket.emit('cycle_ready')


def handle_world_camera_image_raw(image):
    # TODO : Image will most likely not be sent this way
    json_data = to_json({"table_image": image})
    socket.emit('table_image', json_data)


def websockets():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('websockets', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber("ready", Bool, handle_ready)
    rospy.Subscriber("world_camera/image_raw", Image, handle_world_camera_image_raw)

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
