#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from phoenix6 import swerve
from wpilib import DriverStation
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from ntcore import NetworkTableInstance

# add import for Fuel subsystem
from subsystems.fuel import Fuel
# add import for the new command
from commands.shooter_mode import ShooterMode
from commands.adjust import Adjust
from commands.intake_mode import IntakeMode

import math


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            6
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0)
            .with_rotational_deadband(
                self._max_angular_rate * 0
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        self._logger = Telemetry(self._max_speed)

        # Driver controller (port 0)
        self._joystick = CommandXboxController(0)

        # Operator controller (port 1) - passed into subsystems that need operator inputs
        self._operator = CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        # instantiate fuel subsystem and pass the operator controller so
        # all fuel controls and button bindings are on the same joystick
        self.fuel = Fuel(self._operator)

        # Limelight NetworkTables setup
        nt = NetworkTableInstance.getDefault()
        self._limelight_table = nt.getTable("limelight")

        # Limelight mounting yaw offset (degrees)
        self._limelight_yaw = 10.0

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        #        self.drivetrain.setDefaultCommand(
        #            # Drivetrain will execute this command periodically
        #            self.drivetrain.apply_request(
        #                lambda: (
        #                    self._drive.with_velocity_x(-squaring(self._joystick.getLeftY())
        #                         * self._max_speed
        #                    )  # Drive forward with negative Y (forward)
        #                    .with_velocity_y(-squaring(self._joystick.getLeftX())
        #                         * self._max_speed
        #                    )  # Drive left with negative X (left)
        #                    .with_rotational_rate(-squaring(self._joystick.getRightX())
        #                        * self._max_angular_rate
        #                    )  # Drive counterclockwise with negative X (left)
        #                )
        #            )
        #        )
        # Helper: convert joystick X/Y to polar, scale magnitude, convert back.
        def to_polar(x: float, y: float):
            return (math.hypot(x, y), math.atan2(y, x))

        def from_polar(r: float, theta: float):
            return (r * math.cos(theta), r * math.sin(theta))

        def joystick_request():
            # Map joystick so negative LeftY -> forward, negative LeftX -> left
            jx = -self._joystick.getLeftX()  # left positive
            jy = -self._joystick.getLeftY()  # forward positive

            r, theta = to_polar(jx, jy)
            r = min(1.0, r)
            # Apply the same non-linear scaling you used previously, but to magnitude only
            r_scaled = squaring(r)  # r >= 0 so squaring -> r^2
            jx_s, jy_s = from_polar(r_scaled, theta)

            vx = jy_s * self._max_speed  # forward
            vy = jx_s * self._max_speed  # left
            rot = -squaring(self._joystick.getRightX()) * self._max_angular_rate

            return self._drive.with_velocity_x(vx).with_velocity_y(vy).with_rotational_rate(rot)

        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(lambda: joystick_request()))

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        # Hold A on the operator controller: enter shooter mode (velocity control
        # for shooters + intake/kicker/feed directions)
        SHOOTER_TARGET_RPM = 4100.0 # adjust to your desired velocity setpoint (RPM) 
       
        #80 inches minimum distance away = 3900 rpm
        #129 inches minimum distance away = 4500 rpm
        #153 inches minimum distance away = 5000 rpm
        #98 inches minimum distance away = 4100 rpm
        self._operator.a().whileTrue(ShooterMode(self.fuel, SHOOTER_TARGET_RPM))

        # Hold B on the operator controller: Adjust -> shoot in opposite direction
        self._operator.b().whileTrue(Adjust(self.fuel))

        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def periodic(self) -> None:
        """
        Called every robot frame (20 ms) from robotPeriodic.
        Publishes robot orientation to the Limelight for MegaTag2,
        and sets the Limelight camera pose.
        """
        # Get robot yaw from the Pigeon 2 via the drivetrain's pose
        # The CTRE swerve drivetrain tracks pose using the Pigeon 2 internally.
        robot_yaw_deg = self.drivetrain.get_state().pose.rotation().degrees()

        # MegaTag2 requirement: SetRobotOrientation every frame
        # LimelightHelpers equivalent via NetworkTables:
        # Key: "robot_orientation_set" -> [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        self._limelight_table.getEntry("robot_orientation_set").setDoubleArray(
            [robot_yaw_deg, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

        # Also set the camera mounting pose (including our 10-degree yaw offset)
        # Key: "camerapose_robotspace_set" -> [x, y, z, roll, pitch, yaw]
        self._limelight_table.getEntry("camerapose_robotspace_set").setDoubleArray(
            [0.0, 0.0, 0.0, 0.0, 0.0, self._limelight_yaw]
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        idle = swerve.requests.Idle()
        SHOOTER_TARGET_RPM = 4100.0

        return cmd.sequence(
            # 0. Seed field-centric heading (zero the drivetrain)
            self.drivetrain.runOnce(
                lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
            ),

            # 1. Shoot while stationary
            cmd.parallel(
                self.drivetrain.apply_request(lambda: idle),
                ShooterMode(self.fuel, SHOOTER_TARGET_RPM).withTimeout(5.0),
            ),

            # 2. Idle for the rest of autonomous
            self.drivetrain.apply_request(lambda: idle),
        )


def squaring(x):
    return abs(x)*(x)