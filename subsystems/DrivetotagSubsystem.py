import commands2
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.DriveSubsystem import DriveSubsystem

class DriveToTagCommand(commands2.Command):
    def __init__(self, drive: DriveSubsystem, limelight: LimelightSubsystem, distance_target_m: float = 1.0):
        super().__init__()
        self.drive = drive
        self.limelight = limelight
        self.distance_target_m = distance_target_m

        self.addRequirements(drive, limelight)

    def execute(self):
        forward_cmd = 0.0
        turn_cmd = 0.0

        if not self.limelight.has_target():
            self.drive.drive(0.0, 0.0, 0.0, False)
            return

        # Turn to the tags
        tx = self.limelight.get_horizontal_offset()
        kP_turn = 0.02
        turn_cmd = -tx * kP_turn

        # Drive forward 
        dist = self.limelight.get_distance_to_tag()
        if dist is None:
            self.drive.drive(0.0, 0.0, 0.0, False)
            return

        kP_drive = 0.7
        forward_cmd = (dist - self.distance_target_m) * kP_drive

        # Clamp outputs
        forward_cmd = max(min(forward_cmd, 1.0), -1.0)
        turn_cmd = max(min(turn_cmd, 1.0), -1.0)

        self.drive.drive(forward_cmd, 0.0, turn_cmd, False)

    def isFinished(self):
        return False

    def end(self, interrupted):
        self.drive.drive(0.0, 0.0, 0.0, False)
