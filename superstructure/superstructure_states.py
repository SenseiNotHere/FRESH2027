from typing import TYPE_CHECKING
from .robot_state import RobotState

if TYPE_CHECKING:
    from .superstructure import Superstructure

class SuperstructureStates:
    # Handles idle state by ensuring all subsystems are stopped.
    def _handle_idle(self: "Superstructure"): # type: ignore
        """
        Handles the idle state by stopping all subsystems.
        """
        self._stop_orchestra()

    # Start song on entry
    def _handle_playing_song(self: "Superstructure"): # type: ignore
        """
        Handles the playing song state by playing the current loaded song.
        """
        if self.hasOrchestra:
            self.orchestra.play_selected_song()

    # Start championship song on entry
    def _handle_playing_championship_song(self: "Superstructure"): # type: ignore
        """
        !! ONLY WHEN CHAMPIONSHIP IS ENABLED !!

        Handles the playing championship song state by playing the championship song.
        """
        if self.hasOrchestra:
            self.orchestra.play_championship_song()
