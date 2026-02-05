import commands2
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, HolonomicDriveController, ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians

from subsystems.DriveSubsystem import DriveSubsystem
from Constants import DriveConstants


class DriveToPoseCommand(commands2.Command):
    """
    Command to drive the robot to a specific Pose2d using HolonomicDriveController.
    """
    def __init__(self, drive: DriveSubsystem, target_pose: Pose2d):
        super().__init__()
        self.drive = drive
        self.target_pose = target_pose

        self.addRequirements(drive)

        # PID controllers for X, Y, and rotation
        self.holo_controller = HolonomicDriveController(
            PIDController(3.0, 0.0, 0.0),  # X
            PIDController(3.0, 0.0, 0.0),  # Y
            ProfiledPIDControllerRadians(
                2.0, 0.0, 0.0,  # P, I, D
            TrapezoidProfileRadians.Constraints(
                DriveConstants.kMaxAngularSpeed,        # max velocity in rad/s
                DriveConstants.kMaxAngularAcceleration  # max acceleration in rad/s^2
                )
            )
        )

    def execute(self):
        current_pose: Pose2d = self.drive.getPose()
        chassis_speeds = self.holo_controller.calculate(
            current_pose,
            self.target_pose,
            self.target_pose.rotation
        )

        # Drive with field-relative speeds
        self.drive.drive(
            chassis_speeds.vx / DriveConstants.kMaxSpeedMetersPerSecond,
            chassis_speeds.vy / DriveConstants.kMaxSpeedMetersPerSecond,
            chassis_speeds.omega / DriveConstants.kMaxAngularSpeed,
            fieldRelative=True
        )

    def isFinished(self) -> bool:
        # Tell the editor that getPose() returns a Pose2d
        current_pose: Pose2d = self.drive.getPose()

        # Tell the editor the types of translation and rotation
        current_translation: Translation2d = current_pose.translation
        target_translation: Translation2d = self.target_pose.translation
        pose_error: float = current_translation.distance(target_translation)

        current_rotation: Rotation2d = current_pose.rotation
        target_rotation: Rotation2d = self.target_pose.rotation
        rot_error: float = abs(current_rotation.radians() - target_rotation.radians())

        return pose_error < 0.1 and rot_error < 0.05

    def end(self, interrupted: bool):
        # Stop the robot when command ends
        self.drive.drive(0.0, 0.0, 0.0, fieldRelative=True)
