"""
from rev import SparkFlex, SparkLowLevel, SparkBase
from commands2 import Subsystem
from Configs import Configs
from Constants import ClimbSubsystemConstants


class ClimbSubsystem(Subsystem):
 
    #Climb subsystem using two linear actuators (SparkFlex).
    #Only supports a single climb height (low bar).
    

    def __init__(self):
        super().__init__()

        # Motors
        self.left_motor = SparkFlex(
            ClimbSubsystemConstants.kLeftMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.right_motor = SparkFlex(
            ClimbSubsystemConstants.kRightMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )

        self.left_motor.configure(
            Configs.ClimbSubsystem.leftConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        self.right_motor.configure(
            Configs.ClimbSubsystem.rightConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        # Encoders
        self.left_encoder = self.left_motor.getEncoder()
        self.right_encoder = self.right_motor.getEncoder()

        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

        # Controllers
        self.left_controller = self.left_motor.getClosedLoopController()
        self.right_controller = self.right_motor.getClosedLoopController()

        # Start stowed
        self.target_position = ClimbSubsystemConstants.Setpoints.kStowed

    def climb(self):
        #Move both actuators to climb position
        self.target_position = ClimbSubsystemConstants.Setpoints.kLow

    def stow(self):
        #Move both actuators to stowed position
        self.target_position = ClimbSubsystemConstants.Setpoints.kStowed

    def stop(self):
        self.left_motor.stopMotor()
        self.right_motor.stopMotor()

    def periodic(self):
        self.left_controller.setReference(
            self.target_position,
            SparkLowLevel.ControlType.kPosition
        )
        self.right_controller.setReference(
            self.target_position,
            SparkLowLevel.ControlType.kPosition
        )
"""
