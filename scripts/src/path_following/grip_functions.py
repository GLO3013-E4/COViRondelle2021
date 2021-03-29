from enum import Enum


class GripFunctions(Enum):
    GRAB = 0
    RELEASE = 1

    def get_mode_mapping(self):
        return {
            self.GRAB: 0,
            self.RELEASE: 0
        }
