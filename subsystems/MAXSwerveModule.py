#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


from rev import SparkMax, SparkLowLevel
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from Configs import Configs
import commands2




class MAXSwerveModule:
    def __init__(self, driving_can_id, turning_can_id, chassis_angular_offset):

        self.m_driving_spark = SparkMax(driving_can_id, SparkLowLevel.MotorType.kBrushless)
        self.m_turning_spark = SparkMax(turning_can_id, SparkLowLevel.MotorType.kBrushless)

        self.m_driving_encoder = self.m_driving_spark.getEncoder()
        self.m_turning_encoder = self.m_turning_spark.getAbsoluteEncoder()

        self.m_driving_closed_loop_controller = self.m_driving_spark.getClosedLoopController()
        self.m_turning_closed_loop_controller = self.m_turning_spark.getClosedLoopController()

        # Apply the respective configurations to the SPARKS
        self.m_driving_spark.configure(Configs.MAXSwerveModule.drivingConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)
        self.m_turning_spark.configure(Configs.MAXSwerveModule.turningConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)

        self.m_chassis_angular_offset = chassis_angular_offset
        self.m_desired_state = SwerveModuleState(0.0, Rotation2d())

        # Initialize the state with the current turning encoder value
        self.m_desired_state.angle = Rotation2d(self.m_turning_encoder.getPosition())
        self.m_driving_encoder.setPosition(0)

    def get_state(self):
        """
        Returns the current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position relative to the chassis.
        return SwerveModuleState(self.m_driving_encoder.getVelocity(),
                                 Rotation2d(self.m_turning_encoder.getPosition() - self.m_chassis_angular_offset))

    def get_position(self):
        """
        Returns the current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position relative to the chassis.
        return SwerveModulePosition(self.m_driving_encoder.getPosition(),
                                    Rotation2d(self.m_turning_encoder.getPosition() - self.m_chassis_angular_offset))

    def set_desired_state(self, desired_state: SwerveModuleState):
        """
        Sets the desired state for the module.
        """

        # Apply chassis angular offset to the desired state.
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle + Rotation2d(self.m_chassis_angular_offset)

        # Optimize the reference state to avoid spinning further than 90 degrees.
        corrected_desired_state.optimize(Rotation2d(self.m_turning_encoder.getPosition()))

        # Command driving and turning SPARKS towards their respective setpoints.
        self.m_driving_closed_loop_controller.setReference(corrected_desired_state.speed, SparkLowLevel.ControlType.kVelocity)
        self.m_turning_closed_loop_controller.setReference(corrected_desired_state.angle.radians(), SparkLowLevel.ControlType.kPosition)

        self.m_desired_state = corrected_desired_state

    def reset_encoders(self):
        """
        Zeroes all the SwerveModule encoders.
        """
        self.m_driving_encoder.setPosition(0) 

