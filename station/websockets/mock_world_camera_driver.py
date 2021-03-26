import rospy
import PIL.Image
from numpy import asarray
from sensor_msgs.msg import Image as ROSImage

STUB_IMAGE_PATH = 'data/stub_table_image.jpg'


# TODO : How the hell can be fake this image?
def create_image_data():
    image = PIL.Image.open(STUB_IMAGE_PATH)
    return asarray(image)


# TODO : Remove this mock
if __name__ == '__main__':
    world_camera_publisher = rospy.Publisher('world_camera/image_raw', ROSImage, queue_size=10)

    rospy.init_node('mock_world_camera_driver', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        world_camera_publisher.publish({}, 0, 0, '', False, 0, create_image_data())
        rate.sleep()
