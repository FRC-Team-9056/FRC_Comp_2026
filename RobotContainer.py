#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from commands2 import RunCommand, WaitCommand, SequentialCommandGroup, SwerveControllerCommand
from commands2.button import CommandXboxController
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
# Our Libaries/functions/constants
from Constants import OIConstants, AutoConstants, DriveConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.DrivetotagSubsystem import DriveToTagCommand
from subsystems.IntakeSubsystem import IntakeSubsystem
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants, RobotConfig
from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds


class RobotContainer:
    """
    Container class for the robot subystems, default commands, simple
    autonomous routines, and controller bindings
    """
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()
        self.m_limelight = LimelightSubsystem(9056)
        self.m_intake = IntakeSubsystem()
        
        self.m_robotDrive.zeroHeading()

        # Load robot config from PathPlanner GUI
        self.robotConfig = RobotConfig.fromGUISettings()

       
        AutoBuilder.configure(

            self.m_robotDrive.getPose,               # Pose supplier
            self.m_robotDrive.resetOdometry,          # Odometry reset

        # ChassisSpeeds supplier (current robot speeds)
            lambda: ChassisSpeeds(0, 0, 0),

        # ChassisSpeeds consumer (drive robot)
            lambda speeds, ff: self.m_robotDrive.drive(
                speeds.vx / DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.vy / DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.omega / DriveConstants.kMaxAngularSpeed,
                fieldRelative=False
            ),

            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),

            self.robotConfig,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            self.m_robotDrive
        )

        # The controller port assignments
        self.m_driverController = CommandXboxController(OIConstants.kDriverControllerPort)
        self.m_operatorController= CommandXboxController(OIConstants.kOperatorControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default comemands
        ## Drive default
        self.m_robotDrive.setDefaultCommand(
            RunCommand(
                lambda: self.m_robotDrive.drive(
                    -self.applyDeadband(self.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -self.applyDeadband(self.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -self.applyDeadband(self.m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    True),
                self.m_robotDrive
            )
        )


    def applyDeadband(self, value, deadband):
        """Applys a deadband to a joystick input"""
        return value if abs(value) > deadband else 0.0

    def configureButtonBindings(self):
        """Configures the default button bindings"""
        # Locks the wheels into an X shape so that they cannot move
        self.m_driverController.rightStick().whileTrue(
            RunCommand(
                lambda: self.m_robotDrive.setX(),
                self.m_robotDrive
            )
        )

        self.m_driverController.a().whileTrue(
            DriveToTagCommand(self.m_robotDrive, self.m_limelight, distance_target_m=1.0)
        )

        self.m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand(
                lambda: self.m_intake.intake(),
                self.m_intake
                 )
        ).onFalse(
            RunCommand(
                lambda: self.m_intake.stop()
            )
        )

        self.m_operatorController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand( lambda: self.m_intake.eject(),
                self.m_intake
            )
        ).onFalse(
            RunCommand(
                lambda: self.m_intake.stop()
            )
        )

    def getAutonomousCommand(self):
        return PathPlannerAuto("AS@Auto")
    '''
    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()
    '''

    