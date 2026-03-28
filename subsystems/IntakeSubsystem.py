from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs

class IntakeSubsystem(Subsystem):
    
    def __init__(self):
        super().__init__()

        # Bottom launcher motor
        self.bottom_motor = SparkMax(
            Constants.LauncherConstants.kBottomMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.bottom_motor.configure(
            Configs.launcherSubsystem.launcherConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        #intake motor
        self.intake_motor = SparkMax(
            Constants.IntakeConstants.kIntakeMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.intake_motor.configure(
            Configs.intakeSubsystem.intakeConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        #conveyor motor
        self.conveyor_motor = SparkMax(
            Constants.ConveyorConstants.kConveyorMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.conveyor_motor.configure(
            Configs.conveyorSubsystem.conveyorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

   
    def intake(self, speed: float = 0.5):
        """Intake."""
        self.intake_motor.set(speed)
        

    def load(self, speed: float = 0.7):
        self.conveyor_motor.set(-speed)
        self.bottom_motor.set(speed)

    def eject(self, speed: float = 0.5):
        """Eject + conveyor toward intake."""
        self.intake_motor.set(-speed)
        self.conveyor_motor.set(speed)

    def stopintake(self):
        """Stop"""
        self.intake_motor.set(0)
    
    def stopload(self):
        self.conveyor_motor.set(0)
        self.bottom_motor.set(0)
        
