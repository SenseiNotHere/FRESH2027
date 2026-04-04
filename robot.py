#!/usr/bin/env python3

import typing
import wpilib
from commands2 import CommandScheduler, TimedCommandRobot, Command

from robot_container import RobotContainer


class FRCRobot(TimedCommandRobot):

    autonomousCommand: typing.Optional[Command] = None
    testCommand: typing.Optional[Command] = None

    def robotInit(self) -> None:
        self.robot_container = RobotContainer(self)

    def disabledInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        self.robot_container.updateAutoPreview()

    def autonomousInit(self) -> None:
        self.autonomousCommand = self.robot_container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.robot_container.autonomousSubsystem.clearAutoPreview()
        
    def teleopPeriodic(self) -> None:
        pass

    def robotPeriodic(self) -> None:
        super().robotPeriodic()

        if self.isSimulation():
            simPhysics = getattr(self.robot_container.vroomvroom, "simPhysics", None)
            if simPhysics:
                simPhysics.periodic()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
        self.testCommand = self.robot_container.getTestCommand()
        if self.testCommand is not None:
            self.testCommand.schedule()

    def testPeriodic(self) -> None:
        pass