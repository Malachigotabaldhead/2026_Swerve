from commands2 import Subsystem
from commands2.button import CommandXboxController

# Use the real rev.SparkFlex with the known constructor signature:
#   SparkFlex(device_id: int, motor_type: SparkFlex.MotorType)
from rev import SparkFlex
import rev
from ntcore import NetworkTableInstance

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

        # --- Shooter closed-loop gains (hard-coded initial guesses, RPM units) ---
        # store as instance attributes so dashboard/periodic can modify/apply them
        self._shooter_kP = 0.00011
        self._shooter_kI = 0.0
        self._shooter_kD = 0.0057
        self._shooter_kFF = 0.0018
        # --- Shooter gain setup ---
        # Use the SparkFlexConfig + configure(...) API to apply PID/FF to the
        # device. The rev binding exposes SparkFlexConfig.closedLoop.P/I/D and
        # velocityFF and SparkFlex.configure(cfg, resetMode, persistMode).
        def _apply_shooter_gains():
            cfg = rev.SparkFlexConfig()
            cfg.closedLoop.P(self._shooter_kP)
            cfg.closedLoop.I(self._shooter_kI)
            cfg.closedLoop.D(self._shooter_kD)
            # velocityFF expects a feed-forward tuned for the closed-loop
            # velocity control (units depend on device; we used RPM-based kFF)
            cfg.closedLoop.velocityFF(self._shooter_kFF)

            # Configure encoder measurement settings for better velocity readings
            cfg.encoder.uvwAverageDepth(2)
            cfg.encoder.uvwMeasurementPeriod(10)

            # Apply to both shooter motors. Use safe reset/persist modes so we
            # don't unintentionally persist parameters to flash during testing.
            for _motor in (self.outside_shooter, self.inside_shooter):
                _motor.configure(
                    cfg,
                    rev.ResetMode.kNoResetSafeParameters,
                    rev.PersistMode.kNoPersistParameters,
                )

        # apply initial gains
        _apply_shooter_gains()
        # stash helper for later use from periodic()
        self._apply_shooter_gains = _apply_shooter_gains
        # --- end shooter gain setup ---

        # NetworkTables: expose shooter control & PID for Elastic/Shuffleboard/etc.
        self._nt = NetworkTableInstance.getDefault()
        self._shooter_table = self._nt.getTable("Shooter")
        # writable entries Elastic can change
        self._shooter_table.getEntry("enabled").setBoolean(False)
        self._shooter_table.getEntry("setpointRPM").setDouble(0.0)
        self._shooter_table.getEntry("kP").setDouble(self._shooter_kP)
        self._shooter_table.getEntry("kI").setDouble(self._shooter_kI)
        self._shooter_table.getEntry("kD").setDouble(self._shooter_kD)
        self._shooter_table.getEntry("kFF").setDouble(self._shooter_kFF)
        # readback entries
        self._shooter_table.getEntry("actualRPM").setDouble(0.0)
        # cache last-seen to detect changes
        self._last_nt = {
            "enabled": False,
            "setpointRPM": 0.0,
            "kP": self._shooter_kP,
            "kI": self._shooter_kI,
            "kD": self._shooter_kD,
            "kFF": self._shooter_kFF,
        }

        # Operator controller (passed-in from RobotContainer)
        # This keeps controller ownership in RobotContainer so button bindings
        # and fuel controls live on the same joystick.
        self._operator = operator

        # Set default command: continuous loop reading triggers and setting motors
        def _default():
            left = self._operator.getLeftTriggerAxis()
            right = self._operator.getRightTriggerAxis()
            power = (left - right) * 0.5  # scaled down by 50%

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
    def start_shooters_velocity(self, rpm: float) -> None:
        """
        Request the shooter motors to run at a velocity setpoint (rpm) using
        the rev SparkFlex closed-loop API via getClosedLoopController().
        """

        # Use the Spark Flex ClosedLoopController API directly.
        self.outside_shooter.getClosedLoopController().setReference(
            rpm, rev.SparkFlex.ControlType.kVelocity
        )
        self.inside_shooter.getClosedLoopController().setReference(
            rpm, rev.SparkFlex.ControlType.kVelocity
        )

    def stop_shooters(self) -> None:
        """Stop shooter motors."""
        self.outside_shooter.set(0.0)
        self.inside_shooter.set(0.0)

    def periodic(self) -> None:
        """
        Poll NetworkTables Shooter entries and apply changes.
        Elastic dashboards can set:
          - Shooter/enabled (boolean)
          - Shooter/setpointRPM (double)
          - Shooter/kP/kI/kD/kFF (doubles)
        """
        t = self._shooter_table

        enabled_entry = t.getEntry("enabled")
        raw_value = enabled_entry.getValue()
        bool_value = enabled_entry.getBoolean(self._last_nt["enabled"])

        enabled = bool_value
        setpoint = t.getEntry("setpointRPM").getDouble(self._last_nt["setpointRPM"])
        kP = t.getEntry("kP").getDouble(self._last_nt["kP"])
        kI = t.getEntry("kI").getDouble(self._last_nt["kI"])
        kD = t.getEntry("kD").getDouble(self._last_nt["kD"])
        kFF = t.getEntry("kFF").getDouble(self._last_nt["kFF"])

        # apply PID/FF updates if changed
        if (kP, kI, kD, kFF) != (
            self._shooter_kP,
            self._shooter_kI,
            self._shooter_kD,
            self._shooter_kFF,
        ):
            self._shooter_kP, self._shooter_kI, self._shooter_kD, self._shooter_kFF = (
                kP,
                kI,
                kD,
                kFF,
            )
            # re-apply updated gains to the hardware controllers
            self._apply_shooter_gains()

        # apply enable/setpoint changes
        if enabled != self._last_nt["enabled"] or setpoint != self._last_nt["setpointRPM"]:
            if enabled:
                # start closed-loop with RPM setpoint
                self.start_shooters_velocity(setpoint)
            else:
                self.stop_shooters()

        # publish actual encoder RPM if enabled. Use the SparkRelativeEncoder
        # getVelocity() API directly (assumed to return RPM). Keep this simple
        # and avoid runtime introspection — fall back to the setpoint if reading
        # the encoder fails.
        actual_rpm = 0.0
        if enabled:
            enc = self.outside_shooter.getEncoder()
            actual_rpm = float(enc.getVelocity())

        t.getEntry("actualRPM").setDouble(actual_rpm)

        # update last seen
        self._last_nt["enabled"] = enabled
        self._last_nt["setpointRPM"] = setpoint
        self._last_nt["kP"] = kP
        self._last_nt["kI"] = kI
        self._last_nt["kD"] = kD
        self._last_nt["kFF"] = kFF