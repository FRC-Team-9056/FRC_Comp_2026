from rev import SparkMax, SparkLowLevel, SparkBase
from commands2 import Subsystem
from Configs import Configs
from Constants import ClimbSubsystemConstants


class ClimbSubsystem(Subsystem):
    """Climb Subsystem with two arms and preset levels"""

    class DisplaySetpoint:
        kStowed = "Rest"
        kLowBar = "Low"
        kMidBar = "Mid"
        kHighBar = "High"

    def __init__(self):
        super().__init__()

        #Motors configure
        self.left_motor = SparkMax(
            ClimbSubsystemConstants.kLeftMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.left_motor.configure(
            Configs.ClimbSubsystem.leftConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        self.right_motor = SparkMax(
            ClimbSubsystemConstants.kRightMotorCanId,
            SparkLowLevel.MotorType.kBrushless
        )
        self.right_motor.configure(
            Configs.ClimbSubsystem.rightConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )


        self.left_encoder = self.left_motor.getEncoder()
        self.right_encoder = self.right_motor.getEncoder()

        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)


        self.left_controller = self.left_motor.getClosedLoopController()
        self.right_controller = self.right_motor.getClosedLoopController()

       
        self.current_target = ClimbSubsystemConstants.Setpoints.kStowed
        self.was_zeroed = False

    
    def move_to_setpoint(self):
        """Moves both arms to the same climb setpoint"""
        self.left_controller.setReference(
            self.current_target,
            SparkLowLevel.ControlType.kMAXMotionPositionControl
        )
        self.right_controller.setReference(
            self.current_target,
            SparkLowLevel.ControlType.kMAXMotionPositionControl
        )

    #Zero
    def zero_on_limit_switch(self):
        """Zero both climber arms using reverse limit switches"""
        left_limit = self.left_motor.getReverseLimitSwitch().get()
        right_limit = self.right_motor.getReverseLimitSwitch().get()

        if not self.was_zeroed and left_limit and right_limit:
            self.left_encoder.setPosition(0)
            self.right_encoder.setPosition(0)
            self.was_zeroed = True

        elif not left_limit or not right_limit:
            self.was_zeroed = False

    #Set Points
    def set_setpoint_command(self, setpoint):
        if setpoint == ClimbSubsystemConstants.Setpoints.kStowed:
            self.current_target = ClimbSubsystemConstants.Setpoints.kStowed
        elif setpoint == ClimbSubsystemConstants.Setpoints.kLowBar:
            self.current_target = ClimbSubsystemConstants.Setpoints.kLowBar
        elif setpoint == ClimbSubsystemConstants.Setpoints.kMidBar:
            self.current_target = ClimbSubsystemConstants.Setpoints.kMidBar
        elif setpoint == ClimbSubsystemConstants.Setpoints.kHighBar:
            self.current_target = ClimbSubsystemConstants.Setpoints.kHighBar

    def stop(self):
        self.left_motor.stopMotor()
        self.right_motor.stopMotor()

    def periodic(self):
        self.move_to_setpoint()
        self.zero_on_limit_switch()
