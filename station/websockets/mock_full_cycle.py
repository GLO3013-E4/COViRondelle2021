import rospy
import time
from std_msgs.msg import String, Bool

from mock.mock_robot_consumption import mock_robot_consumption
from mock.mock_current_step import mock_current_step, Step
from mock.mock_resistance import mock_resistance
from mock.mock_puck_colors import mock_puck_colors
from mock.mock_letters import mock_letters
from mock.mock_puck_is_in_grip import mock_puck_is_in_grip
from mock.mock_puck_is_not_in_grip import mock_puck_is_not_in_grip

rospy.init_node('mock_full_cycle', anonymous=True)

robot_consumption_publisher = rospy.Publisher('robot_consumption', String, queue_size=10)
current_step_publisher = rospy.Publisher('current_step', String, queue_size=10)
resistance_publisher = rospy.Publisher('resistance', String, queue_size=10)
puck_colors_publisher = rospy.Publisher('puck_colors', String, queue_size=10)
letters_publisher = rospy.Publisher('letters', String, queue_size=10)
movement_publisher = rospy.Publisher('movement_vectors_string', String, queue_size=10)


def handle_start_cycle():
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToResistanceStation)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ReadResistance)
    execute_then_sleep(mock_resistance, resistance_publisher)
    execute_then_sleep(mock_puck_colors, puck_colors_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToControlPanel)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ReadControlPanel)
    execute_then_sleep(mock_letters, letters_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToFirstPuckAndGrabFirstPuck)
    execute_then_sleep(mock_puck_is_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToFirstCornerAndReleaseFirstPuck)
    execute_then_sleep(mock_puck_is_not_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToSecondPuckAndGrabSecondPuck)
    execute_then_sleep(mock_puck_is_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToSecondCornerAndReleaseSecondPuck)
    execute_then_sleep(mock_puck_is_not_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToThirdPuckAndGrabThirdPuck)
    execute_then_sleep(mock_puck_is_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToThirdCornerAndReleaseThirdPuck)
    execute_then_sleep(mock_puck_is_not_in_grip, movement_publisher)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.ToSquareCenter)
    execute_then_sleep(mock_current_step, current_step_publisher, Step.CycleEndedAndRedLedOn)


def execute_then_sleep(execute, *args):
    execute(*args)
    time.sleep(1)


def mock_full_cycle():
    mock_robot_consumption(robot_consumption_publisher)

    rospy.Subscriber("start_cycle", Bool, handle_start_cycle)

    rospy.spin()


if __name__ == '__main__':
    mock_full_cycle()
