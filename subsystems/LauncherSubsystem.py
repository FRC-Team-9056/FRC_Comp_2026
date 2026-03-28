from rev import SparkMax, SparkLowLevel, SparkBase,ResetMode,PersistMode
from commands2 import Subsystem
import Constants
from Configs import Configs


class LauncherSubsystem(Subsystem):
    """Launcher subsystem with 2 motors"""

    def __init__(self):
        super().__init__()

        # Top motors
        self.top_left = SparkMax(
            Constants.LauncherConstants.kTopLeftMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )

        self.top_right = SparkMax(
            Constants.LauncherConstants.kTopRightMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )

        # Configure
        for motor in [self.top_left, self.top_right]:
            motor.configure(
                Configs.launcherSubsystem.launcherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )

        # I think this should be mirroed, because the motors are facing opoosite.
        self.top_right.setInverted(True)

    def spinUp(self, top_speed: float = 1.0):
        self.top_left.set(top_speed)
        self.top_right.set(top_speed)
        

    def stop(self):
        self.top_left.set(0)
        self.top_right.set(0)
   