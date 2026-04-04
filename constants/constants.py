import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.signals import NeutralModeValue
from phoenix6.configs import Slot0Configs

from pathplannerlib.config import RobotConfig


class KrakenX60:
    kFreeSpeedRpm = 5800
    kMaxSpeedMetersPerSecond = 2.8


class DrivingConstants:

    # Robot Geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    halfWheelBase = kWheelBase / 2
    halfTrackWidth = kTrackWidth / 2

    # Module positions
    kFrontLeftX = halfWheelBase
    kFrontLeftY = halfTrackWidth

    kFrontRightX = halfWheelBase
    kFrontRightY = -halfTrackWidth

    kBackLeftX = -halfWheelBase
    kBackLeftY = halfTrackWidth

    kBackRightX = -halfWheelBase
    kBackRightY = -halfTrackWidth

    kModulePositions = [
        Translation2d(kFrontLeftX, kFrontLeftY),
        Translation2d(kFrontRightX, kFrontRightY),
        Translation2d(kBackLeftX, kBackLeftY),
        Translation2d(kBackRightX, kBackRightY),
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)
    
    # Physical Limits
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew Rate Limiting
    kMagnitudeSlewRate = 9.8
    kRotationalSlewRate = 12.0

    # CAN IDs | Drive Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 1
    kBackRightDriving = 3

    # CAN IDs | Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 2
    kBackRightTurning = 4

    # CAN IDs | Absolute Encoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2
    
    # CAN IDs | Odometry
    kPigeonID = 0
    
    # Gyro
    kGyroReversed = -1

    # Lock Deadbands
    kLockVxDeadband = 0.05
    kLockVyDeadband = 0.05
    kLockOmegaDeadband = 0.10


class ModuleConstants:

    # Mechanical
    kDrivingMotorPinionTeeth = 14
    kDrivingMotorReduction = 6.12
    kTurningMotorReduction = 287 / 11.0

    kWheelDiameterMeters = ((0.0965 / 0.97) / 0.98)
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # SDS MK4i L2 Kraken
    kDriveGearRatio = 6.75
    kTurningGearRatio = 150.0 / 7.0

    kWheelRadius = 0.0508

    # Mechanical
    kSlipCurrent = 300.0
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60.0

    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # Module Inversions
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute Encoder Offsets
    kFrontLeftTurningEncoderOffset = 0.264892578125
    kFrontRightTurningEncoderOffset = 0.22265625
    kBackLeftTurningEncoderOffset = -0.080078125
    kBackRightTurningEncoderOffset = 0.09521484375

    # Gains
    kDriveGains = (
        Slot0Configs()
        .with_k_p(0.1)
        .with_k_i(0.0)
        .with_k_d(0.0)
    )

    kTurningGains = (
        Slot0Configs()
        .with_k_p(100.0)
        .with_k_i(0.0)
        .with_k_d(0.0)
    )

    # Encoders
    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kTurningEncoderPositionPIDMinInput = 0.0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    # Neutral Modes
    kDrivingMotorIdleMode = NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = NeutralModeValue(NeutralModeValue.COAST)

    # Current Limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120

    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60

    # Driving Control
    kDrivingMinSpeedMetersPerSecond = 0.01
    kSteerDriveCouplingRatio = 3.857142857142857

    kSteerKs = 0.1
    kSteerHoldDeadband = math.radians(0.25) * (
        1.0 / ((2 * math.pi) / kTurningMotorReduction)
    )

    # Odometry
    kOdometryUpdateFrequency = 250.0 # in Hz

class OIConstants:

    kDriverControllerPort = 0
    kOperatorControllerPort = 1

    kDriveDeadband = 0.05

class AutoConstants:

    config = RobotConfig.fromGUISettings()

    kUseSqrtControl = True

    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    kPController = 1.0
    kPThetaController = 0.5

    kIXController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDThetaController = 0.0

    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )