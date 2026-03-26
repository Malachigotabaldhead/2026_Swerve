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
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import rotationsToRadians
from ntcore import NetworkTableInstance

import math

from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.path import PathPlannerPath

from subsystems.fuel import Fuel
from commands.shooter_mode import ShooterMode
from commands.adjust import Adjust
from commands.intake_mode import IntakeMode
from commands.align_to_hub import AlignToHub


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # Single source of truth for shooter RPM used by teleop and autonomous
    SHOOTER_TARGET_RPM: float = 4100.0

    # Fixed hub positions on the field
    _BLUE_HUB_POSITION = Translation2d(16.54 - 11.91, 4.03)
    _RED_HUB_POSITION = Translation2d(11.91, 4.03)

    @property
    def HUB_POSITION(self) -> Translation2d:
        """
        Return the correct hub position based on the current driver station alliance.
        Defaults to blue if alliance is unknown.
        """
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            return self._RED_HUB_POSITION
        else:
            return self._BLUE_HUB_POSITION

    # Meters to inches conversion factor
    _METERS_TO_INCHES = 39.3701

    def get_shooter_rpm(self) -> float:
        """
        Compute the required shooter RPM based on distance to the hub.
        Uses the equation: v(d) = 0.1007d^2 - 8.631d + 3956.73
        where d is the distance in inches from the robot to the hub.
        """
        distance_m = self._get_distance_to_hub()
        d = distance_m * self._METERS_TO_INCHES  # convert to inches
        rpm = 0.1007 * d * d - 8.631 * d + 3956.73
        return rpm

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

        # --- Hub-facing drive request ---
        # FieldCentricFacingAngle: driver controls translation, heading auto-aims at hub
        self._heading_kP = 15
        self._heading_kI = 2
        self._heading_kD = 1

        self._face_hub = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(0.05)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            # Tune these PID gains for how aggressively the robot turns to face the hub
            # This PID operates on heading radians -> outputs rad/s
            .with_heading_pid(self._heading_kP, self._heading_kI, self._heading_kD)
        )

        self._logger = Telemetry(self._max_speed)

        # Driver controller (port 0)
        self._joystick = CommandXboxController(0)

        # Operator controller (port 1) - passed into subsystems that need operator inputs
        self._operator = CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        # instantiate fuel subsystem and pass the operator controller so
        # all fuel controls and button bindings are on the same joystick
        self.fuel = Fuel(self._operator)

        # Limelight NetworkTables setup (table name must match the Limelight's name)
        nt = NetworkTableInstance.getDefault()
        self._limelight_table = nt.getTable("limelight-bulldog")

        # Configure the button bindings
        self.configureButtonBindings()

        # --- PathPlanner Named Commands ---
        # Register commands that can be triggered from PathPlanner event markers
        NamedCommands.registerCommand(
            "shoot", ShooterMode(self.fuel, self.get_shooter_rpm).withTimeout(9)
        )
        NamedCommands.registerCommand(
            "intake", IntakeMode(self.fuel).withTimeout(9)
        )
        NamedCommands.registerCommand(
            "align", AlignToHub(
                self.drivetrain,
                self._face_hub,
                self._get_angle_to_hub,
                tolerance_deg=3.0,
            ).withTimeout(2.0)
        )

        # --- Auto Chooser ---
        # Build an auto chooser from autos in deploy/pathplanner/autos/
        self._auto_chooser = AutoBuilder.buildAutoChooser()

        # Add mirrored versions of the autos to the chooser
        self._auto_chooser.addOption(
            "New Auto (Mirrored)", self._build_mirrored_auto()
        )
        self._auto_chooser.addOption(
            "Long Auto (Mirrored)", self._build_mirrored_long_auto()
        )

        SmartDashboard.putData("Auto Chooser", self._auto_chooser)

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
        # v(d) = 0.1007d^2 - 8.631d + 3956.73 (d in inches)
        #80 inches minimum distance away = 3900 rpm
        #129 inches minimum distance away = 4500 rpm
        #153 inches minimum distance away = 5000 rpm
        #98 inches minimum distance away = 4100 rpm
        self._operator.a().whileTrue(ShooterMode(self.fuel, self.get_shooter_rpm))

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

        # Hold right bumper on driver controller: drive normally but auto-aim at hub
        def hub_facing_request():
            # Driver still controls translation with joysticks
            jx = -self._joystick.getLeftX()
            jy = -self._joystick.getLeftY()

            r, theta = to_polar(jx, jy)
            r = min(1.0, r)
            r_scaled = squaring(r)
            jx_s, jy_s = from_polar(r_scaled, theta)

            vx = jy_s * self._max_speed
            vy = jx_s * self._max_speed

            # Compute the angle to the hub and set as target direction
            angle_to_hub = self._get_angle_to_hub()

            return (
                self._face_hub
                .with_velocity_x(vx)
                .with_velocity_y(vy)
                .with_target_direction(angle_to_hub)
            )

        self._joystick.rightBumper().whileTrue(
            self.drivetrain.apply_request(lambda: hub_facing_request())
        )

    def periodic(self) -> None:
        """
        Called every robot frame (20 ms) from robotPeriodic.
        Publishes robot orientation to the Limelight for MegaTag2.
        Must be called every frame before reading MegaTag2 pose estimates.
        """
        robot_yaw_deg = self.drivetrain.get_state().pose.rotation().degrees()

        # MegaTag2 requirement: SetRobotOrientation every frame
        # [yaw, yawRate, pitch, pitchRate, roll, rollRate]
        yaw_rate = self.drivetrain._get_robot_yaw_rate_deg_per_sec()
        self._limelight_table.getEntry("robot_orientation_set").setDoubleArray(
            [robot_yaw_deg, yaw_rate, 0.0, 0.0, 0.0, 0.0]
        )

        # Publish hub-tracking debug info
        angle_to_hub = self._get_angle_to_hub()
        distance_to_hub = self._get_distance_to_hub()
        self._limelight_table.getEntry("hub_angle_deg").setDouble(angle_to_hub.degrees())
        self._limelight_table.getEntry("hub_distance_m").setDouble(distance_to_hub)

        # Set Limelight IMU mode based on robot state
        if DriverStation.isDisabled():
            # Seed internal IMU from external while disabled
            self.drivetrain.set_limelight_imu_mode(1)
        else:
            # Use external IMU while enabled
            self.drivetrain.set_limelight_imu_mode(0)

        # Consume MegaTag2 vision pose estimates and feed to pose estimator
        self.drivetrain.update_vision_pose()

        # Flush NT to ensure the Limelight receives the orientation update
        # immediately (matches the Java SetRobotOrientation flush behavior).
        NetworkTableInstance.getDefault().flush()

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.
        Returns the auto selected from the PathPlanner auto chooser on the dashboard.

        PathPlanner's "resetOdom: true" in the .auto file resets the pose
        estimator to the path's starting pose. _should_flip_path() handles
        red alliance mirroring. Do NOT seed field-centric here.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()

    def _build_mirrored_auto(self) -> commands2.Command:
        """
        Build a mirrored version of the 'New New New Path' auto.
        mirrorPath() flips the path across the field's Y-axis midline
        (top <-> bottom of field). Alliance flipping (blue <-> red)
        is still handled automatically by _should_flip_path().
        """
        original_path = PathPlannerPath.fromPathFile("New New New Path")
        mirrored_path = original_path.mirrorPath()

        return cmd.sequence(
            # Reset odometry to the mirrored path's starting pose
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose(
                    mirrored_path.getStartingHolonomicPose()
                )
            ),
            # Run intake in parallel with following the mirrored path
            cmd.parallel(
                AutoBuilder.followPath(mirrored_path),
                IntakeMode(self.fuel).withTimeout(9),
            ),
            # Align to hub AND shoot at the same time
            cmd.parallel(
                AlignToHub(
                    self.drivetrain,
                    self._face_hub,
                    self._get_angle_to_hub,
                    tolerance_deg=3.0,
                ),
                ShooterMode(self.fuel, self.get_shooter_rpm),
            ).withTimeout(9),
        )

    def _build_mirrored_long_auto(self) -> commands2.Command:
        """
        Build a mirrored version of the 'Long' auto.
        mirrorPath() flips the path across the field's Y-axis midline
        (top <-> bottom of field). Alliance flipping (blue <-> red)
        is still handled automatically by _should_flip_path().
        """
        original_path = PathPlannerPath.fromPathFile("Long")
        mirrored_path = original_path.mirrorPath()

        return cmd.sequence(
            # Reset odometry to the mirrored path's starting pose
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose(
                    mirrored_path.getStartingHolonomicPose()
                )
            ),
            # Run intake in parallel with following the mirrored path
            cmd.parallel(
                AutoBuilder.followPath(mirrored_path),
                IntakeMode(self.fuel).withTimeout(9),
            ),
            # Align to hub AND shoot at the same time
            cmd.parallel(
                AlignToHub(
                    self.drivetrain,
                    self._face_hub,
                    self._get_angle_to_hub,
                    tolerance_deg=3.0,
                ),
                ShooterMode(self.fuel, self.get_shooter_rpm),
            ).withTimeout(9),
        )

    def _get_angle_to_hub(self) -> Rotation2d:
        """
        Compute the field-relative angle from the robot's current pose
        to the fixed hub position. Returns a Rotation2d the robot should face.

        Note: FieldCentricFacingAngle applies the operator perspective rotation
        to the target direction. On red alliance the perspective is rotated 180°,
        so we must compensate by adding 180° for red alliance.
        """
        robot_translation = self.drivetrain.get_state().pose.translation()

        # Vector from robot to hub
        dx = self.HUB_POSITION.x - robot_translation.x
        dy = self.HUB_POSITION.y - robot_translation.y

        # atan2(dy, dx) gives the field-relative angle to the hub
        angle_to_hub = Rotation2d(math.atan2(dy, dx))

        # Compensate for red alliance operator perspective (180° rotation)
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            angle_to_hub = angle_to_hub.rotateBy(Rotation2d.fromDegrees(180))

        return angle_to_hub

    def _get_distance_to_hub(self) -> float:
        """
        Return distance in meters from robot to the hub.
        """
        robot_pose = self.drivetrain.get_state().pose
        robot_translation = robot_pose.translation()
        return robot_translation.distance(self.HUB_POSITION)
def squaring(x):
    return abs(x)*(x)