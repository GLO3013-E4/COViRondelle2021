from enum import Enum


class Step(Enum):
    WaitForReadyState = 'WaitForReadyState'
    SendReadyState = 'SendReadyState'
