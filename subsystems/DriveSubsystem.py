import math
import navx
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator
from Constants import DriveConstants
from commands2 import Subsystem
from subsystems.MAXSwerveModule import MAXSwerveModule
from subsystems.LimelightSubsystem import LimelightSubsystem


class DriveSubsystem(Subsystem):
    def __init__(self, limelight=None):
        super().__init__()

        # Create MAX Swerve Modules
        self.m_frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset
        )
        self.m_frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset
        )
        self.m_rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset
        )
        self.m_rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset
        )

        # Gyro
        self.m_gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # Limelight reference
        self.limelight = limelight # this is instance, but if a bug apears, change this to subsystem instead.

        # Pose Estimator
        self.poseEstimator = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ],
            Pose2d()
        )
        self.poseEstimator.setVisionMeasurementStdDevs((0.5, 0.5, math.radians(10)))  # adjust trust

    # Periodic updates
    def periodic(self):
        # Update odometry from gyro
        self.poseEstimator.update(
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ]
        )

        if isinstance(self.limelight, LimelightSubsystem):
            visionPose = self.limelight.get_field_pose()
            if visionPose is not None:
                self.poseEstimator.addVisionMeasurement(visionPose, wpilib.Timer.getFPGATimestamp())


    # Get current estimated pose
    def getPose(self):
        return self.poseEstimator.getEstimatedPosition()

    # Reset odometry
    def resetOdometry(self, pose: Pose2d):
        self.poseEstimator.resetPosition(
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ],
            pose
        )

    # Driving
    def drive(self, xSpeed, ySpeed, rot, fieldRelative=True):
        xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rot * DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromDegrees(-self.m_gyro.getAngle())
                )
            )
        else:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            )

        # Desaturate wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Apply states
        self.m_frontLeft.set_desired_state(swerveModuleStates[0])
        self.m_frontRight.set_desired_state(swerveModuleStates[1])
        self.m_rearLeft.set_desired_state(swerveModuleStates[2])
        self.m_rearRight.set_desired_state(swerveModuleStates[3])

    # Set wheels in X for locking
    def setX(self):
        self.m_frontLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.m_frontRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    # Reset all encoders
    def resetEncoders(self):
        self.m_frontLeft.reset_encoders()
        self.m_rearLeft.reset_encoders()
        self.m_frontRight.reset_encoders()
        self.m_rearRight.reset_encoders()

    # Gyro helpers
    def zeroHeading(self):
        self.m_gyro.zeroYaw()

    def getHeading(self):
        return Rotation2d.fromDegrees(self.m_gyro.getAngle())

    def getTurnRate(self):
        return self.m_gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)
