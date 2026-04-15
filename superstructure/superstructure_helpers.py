from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from superstructure.superstructure import Superstructure

from .robot_state import RobotState


class SuperstructureHelpers:
    def _handle_music_cleanup(self: "Superstructure"): # type: ignore
        if not self.hasOrchestra:
            return
        if self.robot_state != RobotState.PLAYING_SONG and self.hasOrchestra:
            self.orchestra.stop()