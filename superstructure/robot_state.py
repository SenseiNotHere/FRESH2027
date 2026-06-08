from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class RobotState(Enum):

    # General
    IDLE = 0
    PLAYING_SONG = 1
    PLAYING_CHAMPIONSHIP_SONG = 2


@dataclass
class RobotReadiness:
    """Readiness flags used to gate actions. Add fields as subsystems are added."""

    def setRobotReadiness(self, readiness: ReadinessList, value: bool):
        """
        Sets the readiness value for the specified readiness flag.
        """
        setattr(self, readiness.value, value)

    def getRobotReadiness(self, readiness: ReadinessList):
        """
        :return: The current readiness value for the specified readiness flag.
        """
        return getattr(self, readiness.value)


class ReadinessList(Enum):
    """Names of the readiness flags on RobotReadiness. Add entries as needed."""
    pass
