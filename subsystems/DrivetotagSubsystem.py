import commands2
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.DriveSubsystem import DriveSubsystem

class DriveToTagCommand(commands2.Command):
    def __init__(self, drive: "DriveSubsystem", limelight: "LimelightSubsystem", distance_target_m: float = 1.0):
        super().__init__()
        self.drive = drive
        self.limelight = limelight
        self.distance_target_m = distance_target_m

        self.addRequirements(drive)

    def execute(self):
        if not self.limelight.has_target():
            # No tag → stop
            self.drive.drive(forward_cmd, 0.0, turn_cmd, False)
            return

        # Steering toward zero horizontal offset
        tx = self.limelight.get_horizontal_offset()
        kP_turn = 0.02
        turn_cmd = -tx * kP_turn

        # Forward driving until at desired distance
        dist = self.limelight.get_distance_to_tag()
        if dist is None:
            self.drive.drive(0, 0, 0)
            return

        kP_drive = 0.7
        forward_cmd = (dist - self.distance_target_m) * kP_drive

        # Clamp speeds
        forward_cmd = max(min(forward_cmd, 2.0), -2.0)
        turn_cmd = max(min(turn_cmd, 2.0), -2.0)

        self.drive.drive(forward_cmd, 0.0, turn_cmd)

    def isFinished(self):
        if not self.limelight.has_target():
            return True

        dist = self.limelight.get_distance_to_tag()
        if dist is None:
            return True

        return dist <= self.distance_target_m

    def end(self, interrupted):
        self.drive.drive(0, 0, 0)
