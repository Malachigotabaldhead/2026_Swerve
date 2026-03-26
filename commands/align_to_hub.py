from commands2 import Command
from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from typing import Callable


class AlignToHub(Command):
    """
    Rotates the robot in place to face the hub using FieldCentricFacingAngle.
    Translation is zero — the robot only rotates.
    Finishes when the heading error is within tolerance.
    """

    def __init__(
        self,
        drivetrain,
        face_hub_request: swerve.requests.FieldCentricFacingAngle,
        angle_supplier: Callable[[], Rotation2d],
        tolerance_deg: float = 3.0,
    ) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._face_hub = face_hub_request
        self._angle_supplier = angle_supplier
        self._tolerance_deg = tolerance_deg
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        angle_to_hub = self._angle_supplier()
        self._drivetrain.set_control(
            self._face_hub
            .with_velocity_x(0.0)
            .with_velocity_y(0.0)
            .with_target_direction(angle_to_hub)
        )

    def isFinished(self) -> bool:
        # Check if the robot's current heading is within tolerance of the target
        current_heading = self._drivetrain.get_state().pose.rotation()
        target = self._angle_supplier()
        error = abs((current_heading - target).degrees())
        # Normalize to [0, 180]
        if error > 180:
            error = 360 - error
        return error <= self._tolerance_deg

    def end(self, interrupted: bool) -> None:
        self._drivetrain.set_control(swerve.requests.SwerveDriveBrake())