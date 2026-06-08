from typing import TYPE_CHECKING
from commands2.button import CommandGenericHID
from wpilib import Timer, XboxController
from .robot_state import RobotState

if TYPE_CHECKING:
    from .superstructure import Superstructure

class SuperstructureHelpers:
    def _stop_orchestra(self: "Superstructure"): # type: ignore
        if self.hasOrchestra:
            self.orchestra.stop()

    def _handle_music_cleanup(self: "Superstructure"): # type: ignore
        if self.robot_state != RobotState.PLAYING_SONG and self.hasOrchestra:
            self.orchestra.stop()

    @staticmethod
    def _rumble_controller(
            controller: CommandGenericHID | None,
            rumble_type: XboxController.RumbleType,
            rumble_value: float
    ):
        if controller is None:
            return

        controller.getHID().setRumble(
            rumble_type,
            rumble_value
        )

    def _handle_rumble_timeout(self: "Superstructure"): # type: ignore
        if not self._rumble_end_time:
            return

        if Timer.getFPGATimestamp() >= self._rumble_end_time:
            if self.hasDriverController:
                self._rumble_controller(
                    self.driverController,
                    XboxController.RumbleType.kBothRumble,
                    0.0
                )
            self._rumble_end_time = None
