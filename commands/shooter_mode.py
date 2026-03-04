from commands2 import Command
from subsystems.fuel import Fuel

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
        shooter_rpm: float,
        intake_pct: float = -0.2,
        kicker_pct: float = -0.2,
        feed_pct: float = 0.2,
    ) -> None:
        super().__init__()
        self._fuel = fuel
        self._rpm = shooter_rpm
        self._intake = intake_pct
        self._kicker = kicker_pct
        self._feed = feed_pct
        self.addRequirements(fuel)

    def initialize(self) -> None:
        # Start shooter (tries closed-loop velocity, falls back to set())
        self._fuel.start_shooters_velocity(self._rpm)
        # Set other mechanisms (open-loop percent)
        self._fuel.intake.set(self._intake)
        self._fuel.kicker.set(self._kicker)
        self._fuel.feed.set(self._feed)

    def execute(self) -> None:
        # Re-apply to ensure controllers stay commanded
        self._fuel.start_shooters_velocity(self._rpm)
        self._fuel.intake.set(self._intake)
        self._fuel.kicker.set(self._kicker)
        self._fuel.feed.set(self._feed)

    def end(self, interrupted: bool) -> None:
        # Stop all driveables for safety
        self._fuel.stop_shooters()
        self._fuel.intake.set(0.0)
        self._fuel.kicker.set(0.0)
        self._fuel.feed.set(0.0)

    def isFinished(self) -> bool:
        return False