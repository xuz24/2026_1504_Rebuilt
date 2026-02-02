import math

"""Drive-related constants (extracted from the former monolithic constants file).
These are the values that describe chassis geometry, slew rates, and CAN IDs.
"""

# Driving parameters - Note that these are not the maximum capable speeds of
# the robot, rather the allowed maximum speeds
kMaxSpeed = 4.8
kMaxAngularSpeed = 2 * math.pi

kDirectionSlewRate = 1.2  # radians per second
kMagnitudeSlewRate = 1.8  # percent per second (1 = 100%)
kRotationalSlewRate = 2.0  # percent per second (1 = 100%)

# Chassis configuration
kTrackWidth = 0.5715  # Distance between centers of right and left wheels on robot METERS
kWheelBase = 0.5715  # Distance between centers of front and back wheels on robot METERS

# Angular offsets of the modules relative to the chassis in radians
kFrontLeftChassisAngularOffset = -math.pi / 2
kFrontRightChassisAngularOffset = 0
kRearLeftChassisAngularOffset = math.pi
kRearRightChassisAngularOffset = math.pi / 2

# SPARK MAX CAN IDs
kRearRightDrivingCanId = 1
kRearRightTurningCanId = 2

kRearLeftDrivingCanId = 3
kRearLeftTurningCanId = 4

kFrontLeftDrivingCanId = 5
kFrontLeftTurningCanId = 6

kFrontRightDrivingCanId = 7
kFrontRightTurningCanId = 8
