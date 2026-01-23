import math
from rev import SparkMaxConfig, SparkFlexConfig 
from Constants import ModuleConstants

class Configs:
    class MAXSwerveModule:
        drivingConfig = SparkMaxConfig()
        turningConfig = SparkMaxConfig()


        @staticmethod
        def initialize():
            # Use module constants to calculate conversion factors and feed forward gain.
            drivingFactor = ModuleConstants.kWheelDiameterMeters * math.pi / ModuleConstants.kDrivingMotorReduction
            turningFactor = 2 * math.pi
            drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps


            # Configure driving motor settings
            Configs.MAXSwerveModule.drivingConfig \
                .smartCurrentLimit(50) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.MAXSwerveModule.drivingConfig.encoder \
                .positionConversionFactor(drivingFactor) \
                .velocityConversionFactor(drivingFactor / 60.0)  # meters and meters per second
            Configs.MAXSwerveModule.drivingConfig.closedLoop \
                .pid(0, 0, 0) \
                .velocityFF(drivingVelocityFeedForward) \
                .outputRange(-1, 1)


            # Configure turning motor settings
            Configs.MAXSwerveModule.turningConfig \
                .smartCurrentLimit(20) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake)
            Configs.MAXSwerveModule.turningConfig.absoluteEncoder \
                .inverted(True) \
                .positionConversionFactor(turningFactor) \
                .velocityConversionFactor(turningFactor / 60.0)  # radians and radians per second
            Configs.MAXSwerveModule.turningConfig.closedLoop \
                .pid(1, 0, 0) \
                .outputRange(-1, 1) \
                .positionWrappingEnabled(True) \
                .positionWrappingInputRange(0, turningFactor) \
                .setFeedbackSensor(Configs.MAXSwerveModule.turningConfig.closedLoop.FeedbackSensor.kAbsoluteEncoder) # this might need to change to Primary encoder for the feedback loop, but I doubt it.closedLoop.positionWrappingInputRange(0, turningFactor)

    class intakeSubsystem:
        intakeConfig = SparkMaxConfig()

        @staticmethod
        def initialize():
        # Configure intake motor
            Configs.intakeSubsystem.intakeConfig \
                .inverted(True) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake) \
                .smartCurrentLimit(40)
            
    class conveyorSubsystem:
        conveyorConfig = SparkMaxConfig()

        @staticmethod
        def initialize():
        # Configure Conveyor motor
            Configs.conveyorSubsystem.conveyorConfig \
                .inverted(True) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake) \
                .smartCurrentLimit(40)
    
    class launcherSubsystem:
        launcherConfig = SparkMaxConfig()

        @staticmethod
        def initialize():
        # Configure Launcher motor
            Configs.launcherSubsystem.launcherConfig \
                .inverted(True) \
                .setIdleMode(idleMode=SparkMaxConfig.IdleMode.kBrake) \
                .smartCurrentLimit(40)
            
# Call initialization functions
Configs.MAXSwerveModule.initialize()

