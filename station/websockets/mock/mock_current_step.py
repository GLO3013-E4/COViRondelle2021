import random
from enum import Enum

import rospy
from std_msgs.msg import String


class Step(Enum):
    CycleNotStarted = 'CycleNotStarted'
    CycleReadyInWaitingMode = 'CycleReadyInWaitingMode'
    CycleStarted = 'CycleStarted'
    ToResistanceStation = 'ToResistanceStation'
    ReadResistance = 'ReadResistance'
    ToControlPanel = 'ToControlPanel'
    ReadControlPanel = 'ReadControlPanel'
    ToFirstPuckAndGrabFirstPuck = 'ToFirstPuckAndGrabFirstPuck'
    ToFirstCornerAndReleaseFirstPuck = 'ToFirstCornerAndReleaseFirstPuck'
    ToSecondPuckAndGrabSecondPuck = 'ToSecondPuckAndGrabSecondPuck'
    ToSecondCornerAndReleaseSecondPuck = 'ToSecondCornerAndReleaseSecondPuck'
    ToThirdPuckAndGrabThirdPuck = 'ToThirdPuckAndGrabThirdPuck'
    ToThirdCornerAndReleaseThirdPuck = 'ToThirdCornerAndReleaseThirdPuck'
    ToSquareCenter = 'ToSquareCenter'
    CycleEndedAndRedLedOn = 'CycleEndedAndRedLedOn'


def create_current_step():
    return random.choice(list(Step)).name


def mock_current_step(step=create_current_step()):
    current_step_publisher = rospy.Publisher('current_step', String, queue_size=10)

    rospy.loginfo('Mocking current_step: {}'.format(step))
    current_step_publisher.publish(step)


if __name__ == '__main__':
    mock_current_step()
