import ntcore
from wpimath.geometry import Pose3d, Rotation3d, Pose2d, Rotation2d
from commands2 import Subsystem

class LimelightSubsystem(Subsystem):
    """Limelight subsystem"""
    
    def __init__(self, team_number: int):
        super().__init__()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.setServerTeam(team_number)
        inst.startClient4("limelight-client")

        self.table = inst.getTable("limelight")

    #Target detection
    def has_target(self) -> bool:
        """Returns True if at least one target is visible"""
        return self.table.getEntry("tv").getDouble(0) == 1

    def get_visible_tag_ids(self) -> list[int]:
        """Returns a list of tag IDs currently visible"""
        ids = self.table.getEntry("tid").getDoubleArray([])
        return [int(tag_id) for tag_id in ids[:2]]  # only max 2 tags

    def get_visible_tag_poses(self) -> list[tuple[int, Pose3d]]:
        """
        Returns (tag_id, Pose3d) for visible tags
        """
        poses = []

        if not self.has_target():
            return poses

        # Get fused robot pose from Limelight
        arr = self.table.getEntry("botpose_wpiblue").getDoubleArray([])
        if len(arr) < 6:
            return poses

        # Extract values directly from the array
        x = arr[0]
        y = arr[1]
        z = arr[2]
        roll = arr[3]
        pitch = arr[4]
        yaw = arr[5]

        fused_pose = Pose3d(x, y, z, Rotation3d(roll, pitch, yaw))

        # Get visible tag IDs
        tag_ids = self.get_visible_tag_ids()
        for tag_id in tag_ids:
            poses.append((tag_id, fused_pose))

        return poses

    def get_field_pose(self) -> Pose2d | None:
        tag_poses = self.get_visible_tag_poses()
        if not tag_poses:
            return None

        arr = self.table.getEntry("botpose_wpiblue").getDoubleArray([])
        if len(arr) < 6:
            return None

        x = arr[0]
        y = arr[1]
        yaw = arr[5]  # radians

        return Pose2d(x, y, Rotation2d(yaw))


    def get_distance_to_tag(self) -> float | None:
        """
        Returns XY-plane distance to the first visible tag.
        """
        pose = self.get_field_pose()
        if pose is None:
            return None
        return (pose.X()**2 + pose.Y()**2) ** 0.5

    def get_horizontal_offset(self) -> float:
        """Returns the horizontal offset (tx) to the first visible target"""
        return self.table.getEntry("tx").getDouble(0)

    def get_vertical_offset(self) -> float:
        """Returns the vertical offset (ty) to the first visible target"""
        return self.table.getEntry("ty").getDouble(0)

