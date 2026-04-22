from wpilib import XboxController
from commands2 import RunCommand
from commands2.button import CommandGenericHID

from commands import ResetSwerveFront, ResetXY

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_container import RobotContainer
    from superstructure import Superstructure


class ButtonBindings:
    def __init__(
            self, 
            robot_container: RobotContainer, 
            superstructure: Superstructure, 
            driver_controller: CommandGenericHID, 
            operator_controller: CommandGenericHID
            ):
        self.robotContainer = robot_container
        self.superstructure = superstructure
        self.driverController = driver_controller
        self.operatorController = operator_controller

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

