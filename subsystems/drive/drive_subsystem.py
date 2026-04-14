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

from constants import SwerveConstants, ModuleConstants


CALIBRATING_DRIVETRAIN = False
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
            SwerveConstants.kFrontLeftTurning,
            SwerveConstants.kFrontLeftDriving,
            SwerveConstants.kFrontLeftTurningEncoder,
            ModuleConstants.kFrontLeftTurningEncoderOffset,
            SwerveConstants.kFrontLeftX,
            SwerveConstants.kFrontLeftY,
            ModuleConstants.kFrontLeftDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        front_right = module_factory.create_module_constants(
            SwerveConstants.kFrontRightTurning,
            SwerveConstants.kFrontRightDriving,
            SwerveConstants.kFrontRightTurningEncoder,
            ModuleConstants.kFrontRightTurningEncoderOffset,
            SwerveConstants.kFrontRightX,
            SwerveConstants.kFrontRightY,
            ModuleConstants.kFrontRightDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        back_left = module_factory.create_module_constants(
            SwerveConstants.kBackLeftTurning,
            SwerveConstants.kBackLeftDriving,
            SwerveConstants.kBackLeftTurningEncoder,
            ModuleConstants.kBackLeftTurningEncoderOffset,
            SwerveConstants.kBackLeftX,
            SwerveConstants.kBackLeftY,
            ModuleConstants.kBackLeftDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        back_right = module_factory.create_module_constants(
            SwerveConstants.kBackRightTurning,
            SwerveConstants.kBackRightDriving,
            SwerveConstants.kBackRightTurningEncoder,
            ModuleConstants.kBackRightTurningEncoderOffset,
            SwerveConstants.kBackRightX,
            SwerveConstants.kBackRightY,
            ModuleConstants.kBackRightDriveMotorInverted,
            ModuleConstants.kTurningMotorInverted,
            ModuleConstants.kTurningEncoderInverted
        )

        drivetrain_constants = (
            SwerveDrivetrainConstants()
            .with_can_bus_name("rio")
            .with_pigeon2_id(SwerveConstants.kPigeonID)
        )

        # Drivetrain Builder
        SwerveDrivetrain.__init__(
            self,
            TalonFX,
            TalonFX,
            CANcoder,
            drivetrain_constants,
            SwerveConstants.kOdometryUpdateFrequency,
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
        self.xy_limit = SlewRateLimiter(SwerveConstants.kMaxMetersPerSecond)
        self.rot_limit = SlewRateLimiter(SwerveConstants.kMaxAngularSpeed)

        # Field 2D
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Alliance
        self.alliance = None
        if DriverStation.Alliance is not None:
            self.alliance = DriverStation.getAlliance()

        # Sim stuff
        self.last_speeds = ChassisSpeeds(0, 0, 0)

    # Periodic
    def periodic(self) -> None:
        pose = self.get_state().pose
        self.field.setRobotPose(pose)
        
        SmartDashboard.putNumber("Drivetrain/X", pose.x)
        SmartDashboard.putNumber("Drivetrain/Y", pose.y)
        SmartDashboard.putNumber("Drivetrain/Heading", pose.rotation().degrees())

        # Drivetrain Calibration
        if CALIBRATING_DRIVETRAIN:
            dist_from_origin = math.hypot(pose.x, pose.y) # distance from origin
            SmartDashboard.putNumber("Drivetrain/Calibrating/DistanceFromOrigin", dist_from_origin)
            # Great used for calibrating the gyro

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
            
        vx = xSpeed * SwerveConstants.kMaxMetersPerSecond
        vy = ySpeed * SwerveConstants.kMaxMetersPerSecond
        omega = rot * SwerveConstants.kMaxAngularSpeed

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