from commands2 import Command
from subsystems.fuel import Fuel
from typing import Callable, Union

class ShooterMode(Command):
    """
    Hold-button command: run shooters (closed-loop if available) and set intake/kicker/feed
    directions while held. Stops all when released.

    Behavior requested:
      - Shooter = Forward (velocity)
      - Intake = Backward (open-loop)
      - Kicker = Backward (open-loop)
      - Feed = Forward (open-loop)
    """

    def __init__(
        self,
        fuel: Fuel,
        shooter_rpm: Union[float, Callable[[], float]],
        intake_pct: float = -1,
        kicker_pct: float = -1,
        feed_pct: float = 1,
    ) -> None:
        super().__init__()
        self._fuel = fuel
        # Accept either a fixed RPM or a callable that returns RPM
        self._rpm_supplier = shooter_rpm if callable(shooter_rpm) else lambda: shooter_rpm
        self._intake = intake_pct
        self._kicker = kicker_pct
        self._feed = feed_pct
        self.addRequirements(fuel)
        # only run other mechanism motors once shooter within this many RPM of setpoint
        self._rpm_tolerance = 1000.0

    def initialize(self) -> None:
        # Start shooter closed-loop velocity
        self._fuel.start_shooters_velocity(self._rpm_supplier())
        # Do NOT start intake/kicker/feed yet — wait for shooter to spin up

    def execute(self) -> None:
        # Recalculate RPM each frame (distance may change)
        rpm = self._rpm_supplier()

        # Always keep commanding the shooter setpoint
        self._fuel.start_shooters_velocity(rpm)

        # Only run intake/kicker/feed when shooter is within tolerance of setpoint
        if abs(self._fuel.get_actual_rpm() - rpm) <= self._rpm_tolerance:
            self._fuel.intake.set(self._intake)
            self._fuel.kicker.set(self._kicker)
            self._fuel.feed.set(self._feed)
        else:
            # Hold other mechanisms stopped while shooter spins up
            self._fuel.intake.set(0.0)
            self._fuel.kicker.set(0.0)
            self._fuel.feed.set(0.0)

    def end(self, interrupted: bool) -> None:
        # Stop all driveables for safety
        self._fuel.stop_shooters()
        self._fuel.intake.set(0.0)
        self._fuel.kicker.set(0.0)
        self._fuel.feed.set(0.0)

    def isFinished(self) -> bool:
        return False