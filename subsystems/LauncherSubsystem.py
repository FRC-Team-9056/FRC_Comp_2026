from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs


class LauncherSubsystem(Subsystem):
    """Launcher subsystem"""

    def __init__(self):
        super().__init__()

        self.launcher_motor = SparkMax(
            Constants.LauncherConstants.kLauncherMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )

        self.launcher_motor.configure(
            Configs.launcherSubsystem.launcherConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

    def spinUp(self, speed: float = 1.0):
        """Shoot"""
        self.launcher_motor.set(speed)

    def stop(self):
        """Stop."""
        self.launcher_motor.set(0)
