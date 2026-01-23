#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import navx
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState, SwerveModulePosition
from Constants import DriveConstants
from commands2 import Subsystem, SwerveControllerCommand
from subsystems.MAXSwerveModule import MAXSwerveModule
import Constants
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
import commands2

class DriveSubsystem(Subsystem):
    def __init__(self):
        # Create MAXSwerveModules
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

        # The gyro sensor
        self.m_gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)

        # Odometry class for tracking robot pose
        self.m_odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ]
        )

    def followTrajectory(self, trajectory):
        theta_controller = ProfiledPIDControllerRadians(
            Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.TrapezoidProfile.Constraints(
                Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                Constants.AutoConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        )
        theta_controller.enableContinuousInput(-3.14, 3.14)

        holonomic_controller = HolonomicDriveController(
            PIDController(Constants.AutoConstants.kPXController, 0, 0),
            PIDController(Constants.AutoConstants.kPYController, 0, 0),
            theta_controller
        )

        return SwerveControllerCommand(
            trajectory,
            self.getPose,  # Get robot's current pose
            DriveConstants.kDriveKinematics,
            holonomic_controller,
            self.setModuleStates,  # Set swerve module states
            [self]
        )
        

    def periodic(self):
        # Update the odometry 
        self.m_odometry.update(
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ]
        )

    def getPose(self):
        return self.m_odometry.getPose()

    def resetOdometry(self, pose: Pose2d):
        self.m_odometry.resetPosition(
            Rotation2d.fromDegrees(-self.m_gyro.getAngle()),
            [
                self.m_frontLeft.get_position(),
                self.m_frontRight.get_position(),
                self.m_rearLeft.get_position(),
                self.m_rearRight.get_position()
            ],
            pose
        )

    def drive(self, xSpeed, ySpeed, rot, fieldRelative=True):
        # Convert the commanded speeds into the correct units 
        xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rot * DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(-self.m_gyro.getAngle())
                )
            )
        else:
            swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
            )
        
        # Desaturate wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Set the desired state for each swerve module
        self.m_frontLeft.set_desired_state(swerveModuleStates[0])
        self.m_frontRight.set_desired_state(swerveModuleStates[1])
        self.m_rearLeft.set_desired_state(swerveModuleStates[2])
        self.m_rearRight.set_desired_state(swerveModuleStates[3])

    def setX(self):
        self.m_frontLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.m_frontRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.m_rearRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates):
        # Desaturate wheel speeds
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)

        # Set the desired state for each swerve module
        self.m_frontLeft.set_desired_state(desiredStates[0])
        self.m_frontRight.set_desired_state(desiredStates[1])
        self.m_rearLeft.set_desired_state(desiredStates[2])
        self.m_rearRight.set_desired_state(desiredStates[3])

    def resetEncoders(self):
        self.m_frontLeft.reset_encoders()
        self.m_rearLeft.reset_encoders()
        self.m_frontRight.reset_encoders()
        self.m_rearRight.reset_encoders()

    def zeroHeading(self):
        self.m_gyro.zeroYaw()

    def getHeading(self):
        return Rotation2d.fromDegrees(-self.m_gyro.getAngle())
    
    def getTurnRate(self):
        return self.m_gyro.getRate() * (-1.0 if DriveConstants.kGyroReversed else 1.0)