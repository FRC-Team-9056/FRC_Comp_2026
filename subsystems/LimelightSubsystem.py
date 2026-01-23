import ntcore
from wpimath.geometry import Pose3d, Rotation3d
from commands2 import Subsystem

class LimelightSubsystem(Subsystem):
    """Limelight subsystem"""
    
    def __init__(self, team_number: int):
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.setServerTeam(team_number)
        inst.startClient4("limelight-client")

        self.table = inst.getTable("limelight")

    def has_target(self) -> bool:
        return self.table.getEntry("tv").getDouble(0) == 1

    def get_tag_id(self) -> int:
        return int(self.table.getEntry("tid").getDouble(-1))

    def get_robot_pose(self) -> Pose3d | None:
         if not self.has_target():
            return None

         arr = self.table.getEntry("botpose_wpiblue").getDoubleArray([])

         if len(arr) < 6:
            return None

         x, y, z, roll, pitch, yaw = arr[:6]
         return Pose3d(x, y, z, Rotation3d(roll, pitch, yaw))

    def get_distance_to_tag(self) -> float | None:
        pose = self.get_robot_pose()
        if pose is None:
            return None

        # Distance in XY 
        return (pose.X()**2 + pose.Y()**2) ** 0.5

    def get_horizontal_offset(self) -> float:
        return self.table.getEntry("tx").getDouble(0)

    def get_vertical_offset(self) -> float:
        return self.table.getEntry("ty").getDouble(0)
