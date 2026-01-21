from rev import SparkMax, SparkLowLevel,  SparkBase
from commands2 import Subsystem
import Constants
from Configs import Configs

class IntakeSubsystem(Subsystem):
    """Controls the intake motor for picking up and ejecting balls."""
    
    def __init__(self, motor_port: int):
        super().__init__()
        # Create the motor controller 
        self.intake_motor = SparkMax(
            Constants.IntakeConstants.kIntakeMotorCanId,
            SparkLowLevel.MotorType.kBrushless
            )
        self.intake_motor.configure(
            Configs.intakeSubsystem.intakeConfig,
             SparkBase.ResetMode.kResetSafeParameters,
             SparkBase.PersistMode.kPersistParameters)

    def intake(self, speed: float = 1.0):
        """Spin the intake to pick up balls."""
        self.intake_motor.set(speed)  

    def eject(self, speed: float = 1.0):
        """Spin the intake in reverse to eject balls."""
        self.intake_motor.set(-speed)  

    def stop(self):
        """Stop the intake."""
        self.intake_motor.set(0)
