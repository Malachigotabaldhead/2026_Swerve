from commands2 import Command
from subsystems.fuel import Fuel


class IntakeMode(Command):
    """
    Run intake/kicker/feed to pull a game piece in.
    Shooters stay off. Ends when interrupted (e.g. timeout or button release).
    """

    def __init__(
        self,
        fuel: Fuel,
        intake_pct: float = -1,
        kicker_pct: float = 1,
        feed_pct: float = -1,
    ) -> None:
        super().__init__()
        self._fuel = fuel
        self._intake = intake_pct
        self._kicker = kicker_pct
        self._feed = feed_pct
        self.addRequirements(fuel)

    def initialize(self) -> None:
        self._fuel.intake.set(self._intake)
        self._fuel.kicker.set(self._kicker)
        self._fuel.feed.set(self._feed)

    def execute(self) -> None:
        self._fuel.intake.set(self._intake)
        self._fuel.kicker.set(self._kicker)
        self._fuel.feed.set(self._feed)

    def end(self, interrupted: bool) -> None:
        self._fuel.intake.set(0.0)
        self._fuel.kicker.set(0.0)
        self._fuel.feed.set(0.0)

    def isFinished(self) -> bool:
        return False