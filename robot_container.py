from __future__ import annotations

import typing
import wpilib
from commands2 import InstantCommand, Command
from commands2.button import CommandGenericHID
from wpilib import XboxController, SendableChooser, SmartDashboard

from pykit.logger import Logger
from pykit.inputs.loggablepowerdistribution import LoggedPowerDistribution

from commands import HolonomicDrive

from constants import *

from superstructure import Superstructure
from subsystems import (
    DriveSubsystem,
    AutonomousSubsystem,
    OrchestraSubsystem,
)

from button_bindings import ButtonBindings

from utils import log, print_banner


class _RobotPowerDistribution(LoggedPowerDistribution):
    """
    pykit's Logger logs the PDH every loop via LoggedPowerDistribution.getInstance(),
    which otherwise hardcodes a REV module at CAN ID 1 and ignores our config. We seed
    the singleton with this subclass so the logger uses our CAN ID/type, skip the
    hardware entirely in simulation, and log defensively so an unresponsive device or a
    missing per-channel current frame can't stop logging or crash the loop.

    Note: WPILib reports CAN read failures as printed HAL warnings, not Python
    exceptions, so try/except can't silence them. To stop persistent per-channel
    "CAN: Message not Found" spam, set RobotConstants.kLogPDHChannels = False.
    """

    def __init__(self, moduleId: int, moduleType: wpilib.PowerDistribution.ModuleType):
        self._available = wpilib.RobotBase.isReal() and RobotConstants.kEnablePDHLogging
        if self._available:
            super().__init__(moduleId, moduleType)

    def saveToTable(self, table):
        if not self._available:
            return

        # Aggregate stats come from a single, always-present status frame; if these
        # fail the device has genuinely dropped, so stop logging entirely.
        try:
            table.put("Voltage", self.distribution.getVoltage())
            table.put("TotalCurrent", self.distribution.getTotalCurrent())
            table.put("TotalPower", self.distribution.getTotalPower())
            table.put("TotalEnergy", self.distribution.getTotalEnergy())
            table.put("Temperature", self.distribution.getTemperature())
        except Exception:
            self._available = False
            return

        if not RobotConstants.kLogPDHChannels:
            return

        # Per-channel currents arrive in separate frames that can be missing right
        # after a reflash or on a firmware/REVLib mismatch; read best-effort.
        channelCurrents = []
        for channel in range(self.distribution.getNumChannels()):
            try:
                channelCurrents.append(self.distribution.getCurrent(channel))
            except Exception:
                channelCurrents.append(0.0)

        table.put("ChannelCurrentsList", channelCurrents)
        table.put("ChannelCurrentsTotal", sum(channelCurrents))


class RobotContainer:
    def __init__(self):
        print_banner("INITIALIZING ROBOT CONTAINER")

        # Controller
        self.driver_controller = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operator_controller = CommandGenericHID(OIConstants.kOperatorControllerPort)

        LoggedPowerDistribution.instance = _RobotPowerDistribution(
            RobotConstants.kPDHCanID, wpilib.PowerDistribution.ModuleType.kRev
        )

        # Subsystems
        def slowdown_when():
            return 0.5 if self.driver_controller.getRawAxis(XboxController.Axis.kLeftTrigger) < 0.5 else 1.0

        self.drive_subsystem = DriveSubsystem(maxSpeedScaleFactor=slowdown_when)
        self.autonomous_subsystem = AutonomousSubsystem(self.drive_subsystem)
        self.orchestra = OrchestraSubsystem(self.drive_subsystem)

        # Superstructure - MUST BE LAST TO INITIALIZE
        self.superstructure = Superstructure(
            drivetrain=self.drive_subsystem,
            orchestra=self.orchestra,
            driverController=self.driver_controller,
            operatorController=self.operator_controller,
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

    def update(self):
        """Called every loop in robotPeriodic. Logs high-level robot state."""
        Logger.recordOutput("Robot/MatchTime", wpilib.Timer.getMatchTime())
        Logger.recordOutput("Robot/BatteryVoltage", wpilib.RobotController.getBatteryVoltage())
        Logger.recordOutput("Robot/RobotMode", RobotConstants.kRobotMode.value)
        Logger.recordOutput("Robot/Enabled", wpilib.DriverStation.isEnabled())
        Logger.recordOutput("Robot/Autonomous", wpilib.DriverStation.isAutonomous())
        Logger.recordOutput("Robot/Teleop", wpilib.DriverStation.isTeleop())
        Logger.recordOutput("Robot/ActiveAuto", str(self._lastPreviewedAuto))

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