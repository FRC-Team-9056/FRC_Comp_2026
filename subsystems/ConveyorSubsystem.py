from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs

class ConveyorSubsystem(Subsystem):
    """Controls the conveyor motor for moving balls from 2 different systems."""
    
    def __init__(self):
        super().__init__()
        # Create the motor controller 
        self.conveyor_motor = SparkMax(
            Constants.ConveyorConstants.kConveyorMotorCanId,
            SparkLowLevel.MotorType.kBrushless
            )
        self.conveyor_motor.configure(
            Configs.conveyorSubsystem.conveyorConfig,
             SparkBase.ResetMode.kResetSafeParameters,
             SparkBase.PersistMode.kPersistParameters)

    def load(self, speed: float = 1.0):
        """Conveyor belt to the launcher."""
        self.conveyor_motor.set(speed)  

    def unload(self, speed: float = 1.0):
        """Conveyor belt to the intake."""
        self.conveyor_motor.set(-speed)  

    def stop(self):
        """Stop."""
        self.conveyor_motor.set(0)

