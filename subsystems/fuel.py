from commands2 import Subsystem
from commands2.button import CommandXboxController

# Use the real rev.SparkFlex with the known constructor signature:
#   SparkFlex(device_id: int, motor_type: SparkFlex.MotorType)
from rev import SparkFlex
import rev

# Choose motor type explicitly at compile time (change to kBrushed if needed)
_SPARKFLEX_MOTOR_TYPE = SparkFlex.MotorType.kBrushless


class Fuel(Subsystem):
    """
    Subsystem 'fuel' using Spark Flex controllers (rev.SparkFlex).
    Device IDs 14..18 -> Bottom Shooter, Top Shooter, Intake, Kicker, Feed
    Both shooters spin in the same direction (both inverted).
    """

    def __init__(self, operator: CommandXboxController):
        super().__init__()

        # Instantiate the real SparkFlex objects with explicit motor type
        self.bottom_shooter = SparkFlex(14, _SPARKFLEX_MOTOR_TYPE)
        self.top_shooter = SparkFlex(15, _SPARKFLEX_MOTOR_TYPE)
        self.intake = SparkFlex(16, _SPARKFLEX_MOTOR_TYPE)
        self.kicker = SparkFlex(17, _SPARKFLEX_MOTOR_TYPE)
        self.feed = SparkFlex(18, _SPARKFLEX_MOTOR_TYPE)

        # Both shooters inverted so they spin in the same correct direction
        self.bottom_shooter.setInverted(True)
        self.top_shooter.setInverted(True)

        # --- Shooter closed-loop gains (applied once at startup) ---
        shooter_kP = 0.00011
        shooter_kI = 0.0
        shooter_kD = 0.0057
        shooter_kFF = 0.0018

        cfg = rev.SparkFlexConfig()
        cfg.closedLoop.P(shooter_kP)
        cfg.closedLoop.I(shooter_kI)
        cfg.closedLoop.D(shooter_kD)
        cfg.closedLoop.velocityFF(shooter_kFF)

        cfg.encoder.uvwAverageDepth(2)
        cfg.encoder.uvwMeasurementPeriod(10)

        # Apply to both shooter motors independently
        for motor in (self.bottom_shooter, self.top_shooter):
            motor.configure(
                cfg,
                rev.ResetMode.kNoResetSafeParameters,
                rev.PersistMode.kNoPersistParameters,
            )
        # --- end shooter gain setup ---

        # Operator controller (passed-in from RobotContainer)
        # This keeps controller ownership in RobotContainer so button bindings
        # and fuel controls live on the same joystick.
        self._operator = operator

        # Set default command: continuous loop reading triggers and setting motors
        def _default():
            left = self._operator.getLeftTriggerAxis()
            right = self._operator.getRightTriggerAxis()
            power = (left - right) * 1  

            # Shooter motors remain zero
            self.bottom_shooter.set(0.0)
            self.top_shooter.set(0.0)

            # Intake runs backward, Kicker forward, Feed backward (signs applied)
            self.intake.set(-power)
            self.kicker.set(power)
            self.feed.set(-power)

        # Register the default command (runs every scheduler tick)
        self.setDefaultCommand(self.run(_default))

    # --- helper methods for the shooter command to call ---
    def start_shooters_velocity(self, rpm: float) -> None:
        """
        Request both shooter motors to run at a velocity setpoint (rpm) using
        the rev SparkFlex closed-loop API via getClosedLoopController().
        Both motors are commanded independently.
        """
        self.bottom_shooter.getClosedLoopController().setReference(
            rpm, rev.SparkFlex.ControlType.kVelocity
        )
        self.top_shooter.getClosedLoopController().setReference(
            rpm, rev.SparkFlex.ControlType.kVelocity
        )

    def stop_shooters(self) -> None:
        """Stop both shooter motors."""
        self.bottom_shooter.set(0.0)
        self.top_shooter.set(0.0)

    def get_actual_rpm(self) -> float:
        """
        Return the measured RPM from the top shooter encoder.
        Assumes encoder.getVelocity() returns RPM.
        """
        enc = self.top_shooter.getEncoder()
        return float(enc.getVelocity())