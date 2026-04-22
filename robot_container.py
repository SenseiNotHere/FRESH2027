from __future__ import annotations

import typing
from commands2 import InstantCommand, Command
from commands2.button import CommandGenericHID
from wpilib import XboxController, SendableChooser, SmartDashboard

from commands import HolonomicDrive

from constants import *

from superstructure import Superstructure
from subsystems import DriveSubsystem, AutonomousSubsystem, OrchestraSubsystem

from button_bindings import ButtonBindings

from utils import log, print_banner


class RobotContainer:
    def __init__(self):
        print_banner("INITIALIZING ROBOT CONTAINER")

        # Controller
        self.driver_controller = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operator_controller = CommandGenericHID(OIConstants.kOperatorControllerPort)

        # Subsystems
        def slowdown_when():
            return 0.5 if self.driver_controller.getRawAxis(XboxController.Axis.kLeftTrigger) < 0.5 else 1.0

        self.drive_subsystem = DriveSubsystem(maxSpeedScaleFactor=slowdown_when)
        self.autonomous_subsystem = AutonomousSubsystem(self.drive_subsystem)
        self.orchestra = OrchestraSubsystem(
            driveSubsystem=self.drive_subsystem,
        )

        # Superstructure - MUST BE LAST TO INITIALIZE
        self.superstructure = Superstructure(
            drivetrain=self.drive_subsystem,
            orchestra=self.orchestra,
            driverController=self.driver_controller,
            operatorController=self.operator_controller

        )

        # Button bindings
        self.button_bindings = ButtonBindings(self, self.superstructure, self.driver_controller, self.operator_controller)
        self.button_bindings.configureButtonBindings()

        self.drive_subsystem.setDefaultCommand(
            HolonomicDrive(
                drivetrain=self.drive_subsystem,
                forwardSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: self.driver_controller.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: self.driver_controller.getRawAxis(XboxController.Axis.kRightX),
                fieldRelative=True,
                rateLimit=True,
                square=True
            )
        )

        # Auto and Test Choosers
        self.auto_chooser = SendableChooser()
        self._lastPreviewedAuto = None
        self.test_chooser = SendableChooser()

        print_banner("ROBOT CONTAINER INITIALIZATION COMPLETE")

    def updateAutoPreview(self):
        selected = self.auto_chooser.getSelected()

        if selected != self._lastPreviewedAuto:
            self.autonomous_subsystem.drawAuto(selected)
            self._lastPreviewedAuto = selected
    
    # Autonomous and Test Command Getters
    def getAutonomousCommand(self) -> typing.Optional[InstantCommand]:
        selected_auto = self.auto_chooser.getSelected()
        if selected_auto is not None:
            log("Robot Container", f"Selected autonomous: {selected_auto.name}")
            return selected_auto.command
        else:
            log("Robot Container", "No autonomous selected")
            return None
        
    def getTestCommand(self) -> typing.Optional[Command]:
        return self.test_chooser.getSelected()