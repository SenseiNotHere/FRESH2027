from __future__ import annotations

import typing

import commands2
from commands2 import InstantCommand
from commands2.button import CommandGenericHID
from wpilib import XboxController, SendableChooser, SmartDashboard

from commands import HolonomicDrive
from constants import OIConstants
from subsystems import DriveSubsystem, AutonomousSubsystem, OrchestraSubsystem
from superstructure.superstructure import Superstructure
from pathplannerlib.auto import AutoBuilder


from utils import log, print_banner


class RobotContainer:
    """
    Minimal robot container with drivetrain only.

    Subsystems Dictionary:
    - vroomvroom: DriveSubsystem
    - smartyPlanner: AutonomousSubsystem
    - musically: OrchestraSubsystem
    - megamente: Superstructure
    """

    def __init__(self, robot):
        print_banner("DRIVETRAIN-ONLY INITIALIZATION STARTING")

        # Drive Subsystem
        log("RobotContainer", "Initializing DriveSubsystem...")
        self.vroomvroom = DriveSubsystem(maxSpeedScaleFactor=lambda: 1.0)


        log("RobotContainer", "DriveSubsystem Initialized!")

        # Autonomous Subsystem
        log("RobotContainer", "Initializing AutonomousSubsystem...")
        self.smartyPlanner = AutonomousSubsystem(self.vroomvroom, self)
        self._lastPreviewedAuto = None
        log("RobotContainer", "AutonomousSubsystem Initialized!")

        # Driver Controller
        log("RobotContainer", "Initializing Driver Controller...")
        self.vroomvroomController = CommandGenericHID(
            OIConstants.kDriverControllerPort
        )
        log("RobotContainer", "Driver Controller Initialized!")

        # Choosers
        log("RobotContainer", "Initializing Choosers...")
        # Auto Chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

        # Test Chooser
        self.testChooser = SendableChooser()
        self.testChooser.setDefaultOption("None", None)
        SmartDashboard.putData("Test Chooser", self.testChooser)
        log("RobotContainer", "Choosers Initialized!")

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

        
        # Orchestra Subsystem
        log("RobotContainer", "Initializing OrchestraSubsystem...")
        self.musically = OrchestraSubsystem(
            driveSubsystem=self.vroomvroom
        )
        log("RobotContainer", "OrchestraSubsystem Initialized!")

        # Superstructure - MUST BE LAST!
        log("RobotContainer", "Initializing Superstructure...")
        self.megamente = Superstructure(
            drivetrain=self.vroomvroom,
            orchestra=self.musically,
            driverController=self.vroomvroomController,
        )
        log("RobotContainer", "Superstructure Initialized!")

        print_banner("ROBOT INITIALIZATION COMPLETE")

    def updateAutoPreview(self):
        selected = self.autoChooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.smartyPlanner.drawAuto(selected)
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