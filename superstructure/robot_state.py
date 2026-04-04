from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):
    # General States
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2

@dataclass
class RobotReadiness:
    pass

class ReadinessList(Enum):
    pass