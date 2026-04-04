from __future__ import annotations

import math

from commands2 import Subsystem
from wpilib import SmartDashboard, Field2d, DriverStation, RobotBase, Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.filter import SlewRateLimiter

from phoenix6.hardware import TalonFX, CANcoder

from phoenix6.swerve import (
    SwerveDrivetrain,
    SwerveDrivetrainConstants,
    SwerveModuleConstantsFactory,
    ClosedLoopOutputType
)

from phoenix6.swerve.requests import ApplyFieldSpeeds, ApplyRobotSpeeds, SwerveDriveBrake

from constants.constants import DrivingConstants, ModuleConstants


class DriveSubsystem(Subsystem, SwerveDrivetrain):
    def __init__(self, maxSpeedScaleFactor):
        Subsystem.__init__(self)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        if self.maxSpeedScaleFactor is not None:
            assert callable(self.maxSpeedScaleFactor)

        # Module factory
        module_factory = (
            SwerveModuleConstantsFactory()
            .with_drive_motor_gear_ratio(ModuleConstants.kDriveGearRatio)
            .with_steer_motor_gear_ratio(ModuleConstants.kTurningGearRatio)
            .with_wheel_radius(ModuleConstants.kWheelRadius)
            .with_slip_current(ModuleConstants.kSlipCurrent)
            .with_drive_motor_gains(ModuleConstants.kDriveGains)
            .with_steer_motor_gains(ModuleConstants.kTurningGains)
            .with_drive_motor_closed_loop_output(ClosedLoopOutputType.TORQUE_CURRENT_FOC)
        )

        front_left = module_factory.create_module_constants(
            DrivingConstants.kFrontLeftDriving,
            DrivingConstants.kFrontLeftTurning,
            DrivingConstants.kFrontLeftTurningEncoder,
            ModuleConstants.kFrontLeftTurningEncoderOffset,
            DrivingConstants.kFrontLeftX,
            DrivingConstants.kFrontLeftY,
            ModuleConstants.kFrontLeftDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )
        
        front_right = module_factory.create_module_constants(
            DrivingConstants.kFrontRightDriving,
            DrivingConstants.kFrontRightTurning,
            DrivingConstants.kFrontRightTurningEncoder,
            ModuleConstants.kFrontRightTurningEncoderOffset,
            DrivingConstants.kFrontRightX,
            DrivingConstants.kFrontRightY,
            ModuleConstants.kFrontRightDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        back_left = module_factory.create_module_constants(
            DrivingConstants.kBackLeftDriving,
            DrivingConstants.kBackLeftTurning,
            DrivingConstants.kBackLeftTurningEncoder,
            ModuleConstants.kBackLeftTurningEncoderOffset,
            DrivingConstants.kBackLeftX,
            DrivingConstants.kBackLeftY,
            ModuleConstants.kBackLeftDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        back_right = module_factory.create_module_constants(
            DrivingConstants.kBackRightDriving,
            DrivingConstants.kBackRightTurning,
            DrivingConstants.kBackRightTurningEncoder,
            ModuleConstants.kBackRightTurningEncoderOffset,
            DrivingConstants.kBackRightX,
            DrivingConstants.kBackRightY,
            ModuleConstants.kBackRightDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        drivetrain_constants = (
            SwerveDrivetrainConstants()
            .with_can_bus_name("rio")
            .with_pigeon2_id(DrivingConstants.kPigeonID)
        )

        # Drivetrain Builder
        SwerveDrivetrain.__init__(
            self,
            TalonFX,
            TalonFX,
            CANcoder,
            drivetrain_constants,
            ModuleConstants.kOdometryUpdateFrequency,
            [
                front_left,
                front_right,
                back_left,
                back_right
            ]
        )
        
        # Requests
        self.field_speeds_request = ApplyFieldSpeeds()
        self.robot_speeds_request = ApplyRobotSpeeds()
        self.brake_request = SwerveDriveBrake()

        # Slew Rate Limiters
        self.xy_limit = SlewRateLimiter(DrivingConstants.kMaxMetersPerSecond)
        self.rot_limit = SlewRateLimiter(DrivingConstants.kMaxAngularSpeed)

        # Field 2D
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Alliance
        self.alliance = None
        if DriverStation.Alliance is not None:
            self.alliance = DriverStation.getAlliance()

        # Speeds for Sim
        self.last_speeds = ChassisSpeeds(0, 0, 0)

    # Periodic
    def periodic(self) -> None:
        pose = self.get_state().pose
        self.field.setRobotPose(pose)
        
        SmartDashboard.putNumber("Drivetrain/X", pose.x)
        SmartDashboard.putNumber("Drivetrain/Y", pose.y)
        SmartDashboard.putNumber("Drivetrain/Heading", pose.rotation().degrees())

    def resetOdometry(self, pose: Pose2d):
        """
        Resets the odometry of the drivetrain to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        self.seed_field_centric(pose.rotation())
        
    def getPose(self) -> Pose2d:
        """
        :return: The current pose of the robot as a Pose2d.
        """
        return self.get_state().pose
    
    def getHeading(self) -> Rotation2d:
        """
        :return: The current heading of the robot as a Rotation2d.
        """
        return self.get_state().pose.rotation()

    def setX(self):
        """
        Sets the robot into X-Break positon.
        """
        self.set_control(self.brake_request)

    def getMotors(self):
        """
        Yields all motors in the drivetrain.
        """
        for module in self.modules:
            yield module.drive_motor
            yield module.steer_motor
            
    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
            square: bool = False
    ):
        if square:
            rot = rot * abs(rot)
            norm = math.hypot(xSpeed, ySpeed)
            xSpeed *= norm
            ySpeed *= norm
            
        vx = xSpeed * DrivingConstants.kMaxMetersPerSecond
        vy = ySpeed * DrivingConstants.kMaxMetersPerSecond
        omega = rot * DrivingConstants.kMaxAngularSpeed

        if rateLimit:
            vx = self.xy_limit.calculate(vx)
            vy = self.xy_limit.calculate(vy)
            omega = self.rot_limit.calculate(omega)
            
        speeds = ChassisSpeeds(vx, vy, omega)
        self.last_speeds = speeds
        
        if fieldRelative:
            self.set_control(self.field_speeds_request.with_speeds(speeds))
        else:
            self.set_control(self.robot_speeds_request.with_speeds(speeds))

    def stop(self):
        self.set_control(self.robot_speeds_request.with_speeds(ChassisSpeeds(0, 0, 0)))
        
    # Autonomous support
    def driveRobotRelativeChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards):

        request = self.robot_speeds_request.with_speeds(speeds)

        if feedforwards is not None:
            request = (
                request
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            )

        self.set_control(request)
        
    def getRobotRelativeSpeeds(self):
        return self.get_state().speeds

    def getAlliance(self):
        if self.alliance is None:
            self.alliance = DriverStation.getAlliance()
        
        return self.alliance

class BadSimPhysics:
    def __init__(self, drivetrain: DriveSubsystem, robot: RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0
        self.x = 0.0
        self.y = 0.0
        self.heading = Rotation2d()

    def periodic(self):  # <-- needs to be indented inside the class
        past = self.t
        self.t = Timer.getFPGATimestamp()

        if past == 0:
            return

        dt = self.t - past

        if not self.robot.isEnabled():
            return

        speeds = self.drivetrain.last_speeds  # instead of get_state().speeds

        self.heading = self.heading + Rotation2d(speeds.omega * dt)
        self.x += speeds.vx * dt
        self.y += speeds.vy * dt

        new_pose = Pose2d(self.x, self.y, self.heading)
        self.drivetrain.reset_pose(new_pose)