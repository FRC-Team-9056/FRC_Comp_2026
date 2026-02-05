from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs

class IntakeSubsystem(Subsystem):
    
    def __init__(self):
        super().__init__()

        self.intake_motor = SparkMax(
            Constants.IntakeConstants.kIntakeMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.intake_motor.configure(
            Configs.intakeSubsystem.intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        self.conveyor_motor = SparkMax(
            Constants.ConveyorConstants.kConveyorMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.conveyor_motor.configure(
            Configs.conveyorSubsystem.conveyorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

   
    def intake(self, speed: float = 1.0):
        """Intake + conveyor toward launcher."""
        self.intake_motor.set(speed)
        self.conveyor_motor.set(speed)

    def eject(self, speed: float = 1.0):
        """Eject + conveyor toward intake."""
        self.intake_motor.set(-speed)
        self.conveyor_motor.set(-speed)

    def stop(self):
        """Stop"""
        self.intake_motor.set(0)
        self.conveyor_motor.set(0)
