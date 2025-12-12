#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from commands2 import RunCommand, StartEndCommand, WaitCommand, SequentialCommandGroup, SwerveControllerCommand, Command, InstantCommand
from commands2.button import CommandXboxController
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
# Our Libaries/functions/constants
from Constants import OIConstants, AutoConstants, DriveConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.MAXSwerveModule import MAXSwerveModule, DriveToTagCommand
from subsystems.LimelightSubsystem import LimelightSubsystem


class RobotContainer:
    """
    Container class for the robot subystems, default commands, simple
    autonomous routines, and controller bindings
    """
    def __init__(self):
        # The robot's subsystems
        self.m_robotDrive = DriveSubsystem()
        self.m_limelight = LimelightSubsystem(9056)
        

        self.m_robotDrive.zeroHeading()

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


    '''
    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()
    '''

class AutonomousCommand:
    def __init__(self, robot_drive: DriveSubsystem):
        self.robot_drive = robot_drive
    
    def get_autonomous_command(self):
        """returns the default autonomous command to run"""
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        config.setKinematics(DriveConstants.kDriveKinematics)
    
        #forward trajectory
        forward_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [],
            Pose2d(1.25, 0, Rotation2d(math.pi)),
            config
        )

        self.robot_drive.resetOdometry(forward_trajectory.initialPose())

        # Create a PIDController for turning
        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController, 0, 0,
            TrapezoidProfileRadians.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        )

        theta_controller.enableContinuousInput(-2 * math.pi, 2 * math.pi)  # Ensure smooth turning
    
        back_config = config
        back_config.setReversed(True)
        #Backward trajectory
        backward_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),  # Start where the previous move ended, but rotated
            [],
            Pose2d(-2.2, 0, Rotation2d(0)),  # Move backward another 1 meter
            back_config
        )
    

        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController, 0, 0,
            TrapezoidProfileRadians.Constraints(
                AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                AutoConstants.kMaxAngularAccelerationRadiansPerSecond
            )
        )

        holonomic_controller = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),  # X control
            PIDController(AutoConstants.kPYController, 0, 0),  # Y control
            theta_controller  # Theta control (ProfiledPIDController)
        )


        forward_command = SwerveControllerCommand(
            forward_trajectory,
            self.robot_drive.getPose,
            DriveConstants.kDriveKinematics,
            holonomic_controller,
            self.robot_drive.setModuleStates,
            [self.robot_drive]
        )
        
        
        backward_command = SwerveControllerCommand(
            backward_trajectory,
            self.robot_drive.getPose,
            DriveConstants.kDriveKinematics,
            holonomic_controller,
            self.robot_drive.setModuleStates,
            [self.robot_drive]
        )

        wait = WaitCommand(5)

        return SequentialCommandGroup(
            # Drive robot backward
            backward_command
        )
