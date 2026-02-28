from commands2 import Subsystem
from commands2.button import CommandXboxController

# Use the real rev.SparkFlex with the known constructor signature:
#   SparkFlex(device_id: int, motor_type: SparkFlex.MotorType)
from rev import SparkFlex

# Choose motor type explicitly at compile time (change to kBrushed if needed)
_SPARKFLEX_MOTOR_TYPE = SparkFlex.MotorType.kBrushless


class Fuel(Subsystem):
    """
    Subsystem 'fuel' using Spark Flex controllers (rev.SparkFlex).
    Device IDs 14..18 -> Outside Shooter, Inside Shooter, Intake, Kicker, Feed
    Outside Shooter is inverted.
    """

    def __init__(self, operator: CommandXboxController):
        super().__init__()

        # Instantiate the real SparkFlex objects with explicit motor type
        self.outside_shooter = SparkFlex(14, _SPARKFLEX_MOTOR_TYPE)
        self.inside_shooter = SparkFlex(15, _SPARKFLEX_MOTOR_TYPE)
        self.intake = SparkFlex(16, _SPARKFLEX_MOTOR_TYPE)
        self.kicker = SparkFlex(17, _SPARKFLEX_MOTOR_TYPE)
        self.feed = SparkFlex(18, _SPARKFLEX_MOTOR_TYPE)

        # Outside Shooter specifically inverted
        self.outside_shooter.setInverted(True)

        # Operator controller (passed-in from RobotContainer)
        # This keeps controller ownership in RobotContainer so button bindings
        # and fuel controls live on the same joystick.
        self._operator = operator

        # Set default command: continuous loop reading triggers and setting motors
        def _default():
            left = self._operator.getLeftTriggerAxis()
            right = self._operator.getRightTriggerAxis()
            power = left - right  # proportional control

            # Shooter motors remain zero
            self.outside_shooter.set(0.0)
            self.inside_shooter.set(0.0)

            # Intake runs backward, Kicker forward, Feed backward (signs applied)
            self.intake.set(-power)
            self.kicker.set(power)
            self.feed.set(-power)

        # Register the default command (runs every scheduler tick)
        self.setDefaultCommand(self.run(_default))

    # --- new helper methods for the shooter command to call ---
    def start_shooters_velocity(self, rps: float) -> None:
        """
        Request the shooter motors to run at a velocity setpoint (rps) using
        the rev SparkFlex velocity API if available, otherwise fall back to set().
        """
        # Try common closed-loop API first
        try:
            from rev import ControlType

            # Many rev APIs use setReference(value, ControlType.kVelocity)
            self.outside_shooter.setReference(rps, ControlType.kVelocity)
            self.inside_shooter.setReference(rps, ControlType.kVelocity)
            return
        except Exception:
            # fall through to open-loop set
            pass

        # Last-resort: open-loop percent/voltage set (caller must choose rps->percent mapping)
        try:
            self.outside_shooter.set(rps)
            self.inside_shooter.set(rps)
        except Exception:
            # let exceptions propagate if hardware API differs unexpectedly
            raise

    def stop_shooters(self) -> None:
        """Stop shooter motors."""
        self.outside_shooter.set(0.0)
        self.inside_shooter.set(0.0)