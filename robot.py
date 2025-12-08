#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
from commands2 import CommandScheduler
from RobotContainer import RobotContainer, AutonomousCommand

class Robot(wpilib.TimedRobot):
    """Sets up the robot and its various modes for competition"""
    def __init__(self):
        super().__init__()
        self.m_autonomousCommand = None
        self.m_robotContainer = None

    def robotInit(self):
        """
        This function is run when the robot is first started up
        and should be used for any initialization code.
        """
        # Instantiate the RobotContainer. This will perform all button bindings and setup.
        self.m_robotContainer = RobotContainer()

    def robotPeriodic(self):
        """This function is called every 20ms. Runs the CommandScheduler."""
        # The Scheduler needs to be run continuously, which will handle commands and subsystems.
        CommandScheduler.getInstance().run()

    def disabledInit(self):
        """Called once when the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self):
        """Called periodically during Disabled mode."""
        pass

    def autonomousInit(self):
        """This function is called once when autonomous mode starts."""
        self.m_autonomousCommand = AutonomousCommand(
            self.m_robotContainer.m_robotDrive
            )
        if hasattr(self.m_autonomousCommand, "get_autonomous_command"):
            self.m_autonomousCommand = self.m_autonomousCommand.get_autonomous_command()

        #Schedule the autonomous command if it exists
        if self.m_autonomousCommand:
            self.m_autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopInit(self):
        """This function is called once when teleop mode starts."""
        # Make sure to cancel the autonomous command if it's running when teleop starts.
        if self.m_autonomousCommand:
            self.m_autonomousCommand.cancel()

    def teleopPeriodic(self):
        """This function is called periodically during teleop mode."""

    def testInit(self):
        """This function is called once at the start of test mode."""
        # Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        pass

    