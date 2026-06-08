from wpilib import XboxController
from commands2 import RunCommand
from commands2.button import CommandGenericHID

from commands import ResetSwerveFront, ResetXY

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_container import RobotContainer
    from superstructure import Superstructure


class ButtonBindings:
    """
    Button bindings for the robot. Connects controllers to the commands that
    operate on the subsystems, kept separate from RobotContainer so the mappings
    stay focused.

    This is a single-instance class. Constructing it a second time raises a
    RuntimeError; use ButtonBindings.getInstance() to reach the existing one.
    """
    _instance: "ButtonBindings | None" = None

    def __init__(
            self,
            robot_container: RobotContainer,
            superstructure: Superstructure,
            driver_controller: CommandGenericHID,
            operator_controller: CommandGenericHID
            ):
        if ButtonBindings._instance is not None:
            raise RuntimeError(
                "ButtonBindings is a single-instance class but was constructed twice. "
                "Use ButtonBindings.getInstance() to reuse the existing instance."
            )

        self.robotContainer = robot_container
        self.superstructure = superstructure
        self.driverController = driver_controller
        self.operatorController = operator_controller

        ButtonBindings._instance = self

    @classmethod
    def getInstance(cls) -> "ButtonBindings":
        if cls._instance is None:
            raise RuntimeError("ButtonBindings has not been constructed yet.")
        return cls._instance

    def configureButtonBindings(self):
        self._configureDriverBindings()

    def _configureDriverBindings(self):
                # Reset Controls
        # Reset XYZ
        self.driverController.pov(0).onTrue(
            ResetXY(
                x=0.0,
                y=0.0,
                headingDegrees=0.0,
                drivetrain=self.robotContainer.drive_subsystem,

            )
        )

        # Reset Robot Front
        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.robotContainer.drive_subsystem)
        )
        
        # X-Break
        self.driverController.button(
            XboxController.Button.kB
        ).whileTrue(
            RunCommand(self.robotContainer.drive_subsystem.setX, self.robotContainer.drive_subsystem)
        )

