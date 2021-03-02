#!/usr/bin/env python
import rospy
from flask import Flask
from flask_socketio import SocketIO
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

# from std_msgs.msg import String

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


def handle_ready(_):
    socketio.emit('cycle_ready')


def handle_world_camera_image_raw(image):
    # TODO : Image will most likely not be sent this way
    # TODO : Format message in JSON
    socketio.emit('table_image', image)


def websockets():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('websockets', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber("ready", Bool, handle_ready)
    rospy.Subscriber("world_camera/image_raw", Image, handle_world_camera_image_raw)

    socketio.run(app)

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
