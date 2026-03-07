from commands2 import Command
from subsystems.fuel import Fuel

class Adjust(Command):
    """
    Hold-button command: run shooters in the opposite direction and flip intake/kicker/feed.
    """

    def __init__(
        self,
        fuel: Fuel,
        shooter_rpm: float = -100.0,       # <-- set default to -100 RPM
        intake_pct: float = 0.5,
        kicker_pct: float = 0.5,
        feed_pct: float = -0.5,
    ) -> None:
        super().__init__()
        self._fuel = fuel
        self._rpm = shooter_rpm
        self._intake = intake_pct
        self._kicker = kicker_pct
        self._feed = feed_pct
        self.addRequirements(fuel)

    def initialize(self) -> None:
        # Shooter reversed using negative RPM (now defaults to -100 RPM)
        self._fuel.start_shooters_velocity(self._rpm)
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
        # Stop everything on release
        self._fuel.stop_shooters()
        self._fuel.intake.set(0.0)
        self._fuel.kicker.set(0.0)
        self._fuel.feed.set(0.0)

    def isFinished(self) -> bool:
        return False