from __future__ import annotations

import typing

import commands2
from commands2 import InstantCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SendableChooser, SmartDashboard

from commands import HolonomicDrive
from constants.constants import OIConstants
from subsystems import DriveSubsystem, BadSimPhysics, AutonomousSubsystem, AutoBuilder
from utils import log, print_banner


class RobotContainer:
    """
    Minimal robot container with drivetrain only.
    """

    def __init__(self, robot):
        print_banner("DRIVETRAIN-ONLY INITIALIZATION STARTING")

        # Drive Subsystem
        log("RobotContainer", "Initializing DriveSubsystem...")
        self.vroomvroom = DriveSubsystem(maxSpeedScaleFactor=lambda: 1.0)

        if commands2.TimedCommandRobot.isSimulation():
            self.vroomvroom.simPhysics = BadSimPhysics(self.vroomvroom, robot)

        log("RobotContainer", "DriveSubsystem Initialized!")

        # Autonomous Subsystem
        log("RobotContainer", "Initializing AutonomousSubsystem...")
        self.autonomousSubsystem = AutonomousSubsystem(self.vroomvroom, self)
        self._lastPreviewedAuto = None
        log("RobotContainer", "AutonomousSubsystem Initialized!")

        # Driver Controller
        log("RobotContainer", "Initializing Driver Controller...")
        self.vroomvroomController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        log("RobotContainer", "Driver Controller Initialized!")

        # Auto Chooser
        log("RobotContainer", "Initializing Auto Chooser...")
        
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        log("RobotContainer", "Auto Chooser Initialized!")

        # Test Chooser
        log("RobotContainer", "Initializing Test Chooser...")
        self.testChooser = SendableChooser()
        self.testChooser.setDefaultOption("None", None)
        SmartDashboard.putData("Test Chooser", self.testChooser)
        log("RobotContainer", "Test Chooser Initialized!")

        # Default Drive Command
        log("RobotContainer", "Setting Default Drive Command...")
        self.vroomvroom.setDefaultCommand(
            HolonomicDrive(
                self.vroomvroom,
                forwardSpeed=lambda: -self.vroomvroomController.getRawAxis(
                    XboxController.Axis.kLeftY
                ),
                leftSpeed=lambda: -self.vroomvroomController.getRawAxis(
                    XboxController.Axis.kLeftX
                ),
                rotationSpeed=lambda: self.vroomvroomController.getRawAxis(
                    XboxController.Axis.kRightX
                ),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=False,
                square=True,
            )
        )
        log("RobotContainer", "Default Drive Command Set!")

        print_banner("ROBOT INITIALIZATION COMPLETE")

    def updateAutoPreview(self):
        selected = self.autoChooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.autonomousSubsystem.drawAuto(selected)
            self._lastPreviewedAuto = selected

    # Autonomous
    def getAutonomousCommand(self) -> commands2.Command:
        command = self.autoChooser.getSelected()

        if command is None:
            log("Autonomous", "WARNING: No autonomous routines selected!")
            return InstantCommand()

        log("Autonomous",f"Running autonomous routine: {command.getName()}")
        return command

    # Test Mode
    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        return self.testChooser.getSelected()