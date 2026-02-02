import wpimath.trajectory as trajectory
"""Autonomous / trajectory constants."""

# NOTE: these values shadow similarly-named drive values in the original file.
# The order of imports in the shim ensures these autos values end up as the final
# names on the `constants` module (preserving previous behavior).
kMaxSpeed = 3  # meters per second
kMaxAcceleration = 2  # meters per second squared
kMaxAngularSpeed = 3.142  # radians per second
kMaxAngularAcceleration = 3.142  # radians per second squared

kPXController = 0.5
kPYController = 0.5
kPThetaController = 0.5

# Trapezoid profile constraints for the theta controller
kThetaControllerConstraints = trajectory.TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAcceleration)
