import rospy
import time
from std_msgs.msg import Bool

# TODO : Fix imports
from mock.mock_robot_consumption import mock_robot_consumption
from mock.mock_current_step import mock_current_step, Step
from mock.mock_resistance import mock_resistance
from mock.mock_puck_colors import mock_puck_colors
from mock.mock_letters import mock_letters
from mock.mock_puck_is_in_grip import mock_puck_is_in_grip
from mock.mock_puck_is_not_in_grip import mock_puck_is_not_in_grip

rospy.init_node('mock_full_cycle', anonymous=True)


def handle_start_cycle():
    execute_then_sleep(mock_current_step, Step.ToResistanceStation)
    execute_then_sleep(mock_current_step, Step.ReadResistance)
    execute_then_sleep(mock_resistance)
    execute_then_sleep(mock_puck_colors)
    execute_then_sleep(mock_current_step, Step.ToControlPanel)
    execute_then_sleep(mock_current_step, Step.ReadControlPanel)
    execute_then_sleep(mock_letters)
    execute_then_sleep(mock_current_step, Step.ToFirstPuckAndGrabFirstPuck)
    execute_then_sleep(mock_puck_is_in_grip)
    execute_then_sleep(mock_current_step, Step.ToFirstCornerAndReleaseFirstPuck)
    execute_then_sleep(mock_puck_is_not_in_grip)
    execute_then_sleep(mock_current_step, Step.ToSecondPuckAndGrabSecondPuck)
    execute_then_sleep(mock_puck_is_in_grip)
    execute_then_sleep(mock_current_step, Step.ToSecondCornerAndReleaseSecondPuck)
    execute_then_sleep(mock_puck_is_not_in_grip)
    execute_then_sleep(mock_current_step, Step.ToThirdPuckAndGrabThirdPuck)
    execute_then_sleep(mock_puck_is_in_grip)
    execute_then_sleep(mock_current_step, Step.ToThirdCornerAndReleaseThirdPuck)
    execute_then_sleep(mock_puck_is_not_in_grip)
    execute_then_sleep(mock_current_step, Step.ToSquareCenter)
    execute_then_sleep(mock_current_step, Step.CycleEndedAndRedLedOn)


def execute_then_sleep(execute, args=None):
    execute(*args)
    time.sleep(1)


def mock_full_cycle():
    rospy.Subscriber("start_cycle", Bool, handle_start_cycle)

    mock_robot_consumption()


if __name__ == '__main__':
    mock_full_cycle()
