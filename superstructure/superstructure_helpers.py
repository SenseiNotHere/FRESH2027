from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from superstructure.superstructure import Superstructure


class SuperstructureHelpers:
    def _handle_music_cleanup(self: "Superstructure"):
        if self.robot_state != RobotState.PLAYING_SONG and self.hasOrchestra:
            self.orchestra.stop()