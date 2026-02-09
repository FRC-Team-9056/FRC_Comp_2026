from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs


class LauncherSubsystem(Subsystem):
    """Launcher subsystem with 3 motors"""

    def __init__(self):
        super().__init__()

        # Bottom motor
        self.bottom_motor = SparkMax(
            Constants.LauncherConstants.kBottomMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )

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
        for motor in [self.bottom_motor, self.top_left, self.top_right]:
            motor.configure(
                Configs.launcherSubsystem.launcherConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )

        # I think this should be mirroed, because the motors are facing opoosite.
        self.top_right.setInverted(True)

    def spinUp(self, top_speed: float = 1.0, bottom_speed: float = 1.0):
        self.top_left.set(top_speed)
        self.top_right.set(top_speed)
        self.bottom_motor.set(bottom_speed)

    def stop(self):
        self.top_left.set(0)
        self.top_right.set(0)
        self.bottom_motor.set(0)
