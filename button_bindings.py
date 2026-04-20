from wpilib import XboxController
from commands2 import RunCommand

from commands import ResetSwerveFront, ResetXY


class ButtonBindings:
    def __init__(self, robot_container):
        self.robotContainer = robot_container

        # Core Subsystems
        self.drivetrain = robot_container.vroomvroom
        self.superstructure = robot_container.megamente
        self.driverController = robot_container.vroomvroomController

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
                drivetrain=self.drivetrain,

            )
        )

        # Reset Robot Front
        self.driverController.pov(180).onTrue(
            ResetSwerveFront(self.drivetrain)
        )
        
        # X-Break
        self.driverController.button(
            XboxController.Button.kB
        ).whileTrue(
            RunCommand(self.drivetrain.setX, self.drivetrain)
        )

