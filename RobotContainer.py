#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from commands2 import RunCommand, WaitCommand, SequentialCommandGroup, SwerveControllerCommand,StartEndCommand
from commands2.button import CommandXboxController
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, TrapezoidProfileRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
# Our Libaries/functions/constants
from Constants import OIConstants, AutoConstants, DriveConstants
from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.DrivetoPoseCommand import DriveToPoseCommand
from subsystems.IntakeSubsystem import IntakeSubsystem
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants, RobotConfig
from wpilib import DriverStation, SmartDashboard,SendableChooser
from wpimath.kinematics import ChassisSpeeds
from subsystems.LauncherSubsystem import LauncherSubsystem
#from subsystems.ClimbSubsystem import ClimbSubsystem
#from Constants import ClimbSubsystemConstants
from pathplannerlib.auto import NamedCommands
from commands2 import InstantCommand



class RobotContainer:
    """
    Container class for the robot subystems, default commands, simple
    autonomous routines, and controller bindings
    """
    def __init__(self):
        # The robot's subsystems(they are being given a name here)
        self.m_limelight = LimelightSubsystem(9056)
        self.m_robotDrive = DriveSubsystem(limelight=self.m_limelight)
        self.m_intake = IntakeSubsystem()
        self.m_lunch = LauncherSubsystem()
        #self.m_climb = ClimbSubsystem()
        
        self.m_robotDrive.zeroHeading()

        # Load robot config from PathPlanner GUI
        self.robotConfig = RobotConfig.fromGUISettings()

        #path planner name commands configuration
        
        NamedCommands.registerCommand(
            "Shoot",
            SequentialCommandGroup(
                InstantCommand(lambda: self.m_lunch.spinUp(), self.m_lunch),
                WaitCommand(0.5),
            )
        )

        NamedCommands.registerCommand(
            "StopShoot",
            InstantCommand(self.m_lunch.stop, self.m_lunch)
        )

        NamedCommands.registerCommand(
            "IntakeOn",
            InstantCommand(lambda: self.m_intake.intake(), self.m_intake)
        )

        NamedCommands.registerCommand(
            "IntakeOff",
            InstantCommand(self.m_intake.stopintake, self.m_intake)
        )

        NamedCommands.registerCommand(
            "Load",
            InstantCommand(lambda: self.m_intake.load(), self.m_intake)
        )

        NamedCommands.registerCommand(
            "Loadstop",
            InstantCommand(self.m_intake.stopload, self.m_intake)
        )


        """
        NamedCommands.registerCommand(
            #"ClimbOn",
            InstantCommand(
                lambda: self.m_climb.climb(), self.m_climb)
        )

        NamedCommands.registerCommand(
            #"ClimbOff",
            InstantCommand(
                lambda: self.m_climb.stow(), self.m_climb)
        )
        """

       #AutoBuilder for pathPlanner(important, don't touch -- Alex)
        AutoBuilder.configure(

            self.m_robotDrive.getPose,               # Pose supplier
            self.m_robotDrive.resetOdometry,          # Odometry reset

        # current robot speeds
            lambda: ChassisSpeeds(0, 0, 0),

        # drive robot
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

              # Autonomous chooser
        self.autoChooser = SendableChooser()

        # Add autos by name 
        self.autoChooser.setDefaultOption("AS@Auto", PathPlannerAuto("AS@Auto"))
        self.autoChooser.addOption("AS@Auto2", PathPlannerAuto("AS@Auto2"))
        self.autoChooser.addOption("AS@Auto3", PathPlannerAuto("AS@Auto3"))
        self.autoChooser.addOption("AS@Auto4", PathPlannerAuto("AS@Auto4"))

        # Put chooser on dashboard
        SmartDashboard.putData("Auto Selector", self.autoChooser)


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

        #Limelight assist
        preset_pose = Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0))
        self.m_driverController.leftBumper().onTrue( 
              DriveToPoseCommand(self.m_robotDrive, preset_pose)
        )

        """
        #Climb
        self.m_operatorController.leftBumper().onTrue(
            RunCommand(
                lambda: self.m_climb.stow(),
                self.m_climb
            )
        )
        
        self.m_operatorController.x().onTrue(
            RunCommand(
                lambda: self.m_climb.climb(),
                self.m_climb
            )
        )
        """

        #Balls Intake
        self.m_operatorController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand(lambda: self.m_intake.intake(),
                self.m_intake
                 )
        ).onFalse(
            InstantCommand(
                lambda: self.m_intake.stopintake(),
                self.m_intake
            )
        )

        #Balls Outtake from the intake
        self.m_operatorController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(
            RunCommand(lambda: self.m_intake.eject(),
                self.m_intake
            )
        ).onFalse(
            InstantCommand(
                lambda: self.m_intake.stopintake(),
                self.m_intake
            )
        )

        #Launcher: load
        self.m_operatorController.rightBumper().whileTrue(
            RunCommand(
                lambda: self.m_intake.load(),
                self.m_intake
            )
        ).onFalse(
            InstantCommand(
                lambda: self.m_intake.stopload(),
                self.m_intake
            )
        )

        #Laucher: shoot
        self.m_operatorController.leftBumper().toggleOnTrue(
            StartEndCommand(
                self.m_lunch.spinUp,   # runs when toggled ON
                self.m_lunch.stop,     # runs when toggled OFF (or canceled)
                self.m_lunch
            )
        )

    #define Autonomous command
    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()
    '''
    def getSimulationTotalCurrentDraw(self):
        # For each subsystem with simulation, returns total current draw
        return self.m_coralSubsystem.get_simulation_current_draw() + self.m_algaeSubsystem.get_simulation_current_draw()
    '''

    