import math
import rev
from rev import SparkMaxConfig

"""Module / swerve-specific constants.
Contains conversion factors, PID gains, idle modes, current limits and absolute encoder offsets.
"""

# Invert the turning encoder, since the output shaft rotates in the opposite direction
# of the steering motor in the MAXSwerve Module
kTurningEncoderInverted = True

# The MaxSwerve module can be configured with one of the three pinion gears: 12T, 13T, or 14T.
# This changes the drive speed of the module ( a pinion gear with more teeth will result in a robot that drives faster)
kDrivingMotorPinionTeeth = 14

# Calculations required for driving motor conversion factors and feed forward
kDrivingMotorFreeSpeedRps = 5676.0 / 60
kWheelDiameter = 0.0762
kWheelCircumference = kWheelDiameter * math.pi

# 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction
kDrivingEncoderPositionFactor = (kWheelDiameter * math.pi) / kDrivingMotorReduction  # Meters
kDrivingEncoderVelocityFactor = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0  # Meters per second

kTurningEncoderPositionFactor = (2 * math.pi)  # radians
kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0  # meters per second

kTurningEncoderPositionPIDMinInput = 0
kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

kDrivingP = 0.04
kDrivingI = 0
kDrivingD = 0
kDrivingFF = (1 / kDriveWheelFreeSpeedRps)
kDrivingMinOutput = -1
kDrivingMaxOutput = 1

kTurningP = 2
kTurningI = 0
kTurningD = 0
kTurningFF = 0
kTurningMinOutput = -1
kTurningMaxOutput = 1

# Idle modes and current limits
kDrivingMotorIdleMode = SparkMaxConfig.IdleMode.kCoast
kTurningMotorIdleMode = SparkMaxConfig.IdleMode.kCoast

kDrivingMotorCurrentLimit = 50  # Amps
kTurningMotorCurrentLimit = 20  # Amps

# Absolute encoder offsets for each module
kRearRightAbsoluteEncoderOffset = 0.1814
kRearLeftAbsoluteEncoderOffset = 0.803
kFrontLeftAbsoluteEncoderOffset = 0.8546
kFrontRightAbsoluteEncoderOffset = 0.665
