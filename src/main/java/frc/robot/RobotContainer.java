// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.commands.*;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RobotBase robot;

  private final CANSparkMax backRightAngleMotor =
      new CANSparkMax(DrivetrainConstants.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
  private final CANSparkMax backRightSpeedMotor =
      new CANSparkMax(DrivetrainConstants.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
  private final CANSparkMax frontRightAngleMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
  private final CANSparkMax frontRightSpeedMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);

  private final CANSparkMax backLeftAngleMotor =
      new CANSparkMax(DrivetrainConstants.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
  private final CANSparkMax backLeftSpeedMotor =
      new CANSparkMax(DrivetrainConstants.BACK_LEFT_SPEED_ID, MotorType.kBrushless);

  private final CANSparkMax frontLeftAngleMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
  private final CANSparkMax frontLeftSpeedMotor =
      new CANSparkMax(DrivetrainConstants.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
  private final WheelSubsystem frontLeftWheel =
      new WheelSubsystem(frontLeftAngleMotor, frontLeftSpeedMotor, "frontLeft");
  private final WheelSubsystem frontRightWheel =
      new WheelSubsystem(frontRightAngleMotor, frontRightSpeedMotor, "frontRight");
  public final WheelSubsystem backLeftWheel =
      new WheelSubsystem(backLeftAngleMotor, backLeftSpeedMotor, "backLeft");
  private final WheelSubsystem backRightWheel =
      new WheelSubsystem(backRightAngleMotor, backRightSpeedMotor, "backRight");
  private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private final NetworkTable table = ntInstance.getTable("/components/Odometry");
  private final NetworkTableEntry inclineAngle = table.getEntry("inclineAngle");
  private final NetworkTableEntry inclineDirection = table.getEntry("inclineDirection");
  private final NetworkTableEntry pitch = table.getEntry("pitch");
  private final NetworkTableEntry roll = table.getEntry("roll");
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final Spark blinkin = new Spark(LedConstants.BLINKIN_CHANNEL);
  private final LedSubsystem ledSubsystem = new LedSubsystem(blinkin);
  private final CANSparkMax pivotMotor =
      new CANSparkMax(ArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
  private final TalonFX telescopeMotor = new TalonFX(ArmConstants.TELESCOPE_MOTOR_ID);
  private final PivotSubsystem pivotSubsystem =
      new PivotSubsystem(pivotMotor, telescopeMotor.getSensorCollection());
  private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem(telescopeMotor);
  private final DoubleSolenoid leftSolenoid =
      new DoubleSolenoid(
          GrabberConstants.PNEUMATICS_HUB_ID,
          PneumaticsModuleType.REVPH,
          GrabberConstants.LEFT_SOLENOID_FORWARD,
          GrabberConstants.LEFT_SOLENOID_REVERSE);

  private final DoubleSolenoid rightSolenoid =
      new DoubleSolenoid(
          GrabberConstants.PNEUMATICS_HUB_ID,
          PneumaticsModuleType.REVPH,
          GrabberConstants.RIGHT_SOLENOID_FORWARD,
          GrabberConstants.RIGHT_SOLENOID_REVERSE);
  private final GrabberSubsystem grabberSubsystem =
      new GrabberSubsystem(leftSolenoid, rightSolenoid);
  private final CANSparkMax elevatorMotor =
      new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorMotor);
  private final HashMap<String, Command> eventMap = new HashMap<>();
  // SENDABLE CHOOSER
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final SwerveModulePosition[] positions =
      new SwerveModulePosition[] {
        frontLeftWheel.getSwerveModulePosition(),
        frontRightWheel.getSwerveModulePosition(),
        backLeftWheel.getSwerveModulePosition(),
        backRightWheel.getSwerveModulePosition()
      };
  final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final SwerveDriveOdometry driveOdometry =
      new SwerveDriveOdometry(
          DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);
  private final Odometry odometry = new Odometry(gyro, driveOdometry, positions, limelight);
  private final SwerveDriveSubsystem swerveDrive =
      new SwerveDriveSubsystem(
          backRightWheel,
          backLeftWheel,
          frontRightWheel,
          frontLeftWheel,
          DrivetrainConstants.SWERVE_KINEMATICS,
          odometry);
  private final Command levelChargingStationCommand =
      ledSubsystem
          .showBalancingColorCommand()
          .andThen(
              new LevelChargingStationCommand(odometry, swerveDrive),
              ledSubsystem.showDockedColorCommand());
  private final AlignByAprilTag alignAtAprilTag =
      new AlignByAprilTag(swerveDrive, limelight, odometry, 0, -0.77); // 1.1);
  private final AlignByAprilTag alignLeftOfAprilTag =
      new AlignByAprilTag(swerveDrive, limelight, odometry, 0.64, -0.77);
  private final AlignByAprilTag alignRightOfAprilTag =
      new AlignByAprilTag(swerveDrive, limelight, odometry, -0.64, -0.77);

  final SlewRateLimiter limitX = new SlewRateLimiter(6);
  final SlewRateLimiter limitY = new SlewRateLimiter(6);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Command followTrajectory(String name, PathConstraints constraints) {
    return new FollowTrajectoryCommand(name, odometry, swerveDrive, eventMap, constraints);
  }

  public Command deliverCone(Constants.ConePosition conePosition) {
    return grabberSubsystem
        .grab()
        .andThen(
            pivotSubsystem
                .moveToTargetContinuously(getArmPositionForCone(conePosition))
                .raceWith(
                    telescopeSubsystem
                        .moveToTargetAndHold(getTelescopePositionForCone(conePosition))
                        .andThen(
                            grabberSubsystem.release(), telescopeSubsystem.moveToTarget(0.03))),
            pivotSubsystem.moveToTargetUntilThere(ArmConstants.elevatorUpPivotDownPosition));
  }

  public RobotContainer(RobotBase robot) {
    this.robot = robot;

    Command balance =
        swerveDrive
            .disableFieldCentricCommand()
            .andThen(
                new PrintCommand("LEVELING"),
                ledSubsystem.showBalancingColorCommand(),
                new LevelChargingStationCommand(odometry, swerveDrive),
                ledSubsystem.showDockedColorCommand());

    Command middleToChargingStationTrajectory =
        followTrajectory("middleToChargingStation", new PathConstraints(4, 4));
    // Move robot from the middle position onto the charging station and balances
    Command middleToChargingStation =
        pivotSubsystem
            .moveToTargetContinuously(ArmConstants.pivotDownPosition)
            .andThen(
                telescopeSubsystem
                    .moveToTarget(0.03)
                    .andThen(
                        middleToChargingStationTrajectory,
                        balance.alongWith(elevatorSubsystem.lowerElevatorCommand())));

    Command chargeCommand =
        followTrajectory("middleToChargingStation", new PathConstraints(2.5, 2.5));

    Command leftToLeftBallBlue =
        followTrajectory("leftToLeftBallBlue", new PathConstraints(2.5, 2.5));
    Command leftToLeftBallRed =
        followTrajectory("leftToLeftBallRed", new PathConstraints(2.5, 2.5));

    Command retrieveLeftBall =
        Commands.either(
                leftToLeftBallRed,
                leftToLeftBallBlue,
                () -> DriverStation.getAlliance() == Alliance.Red)
            .andThen(
                swerveDrive.stop(),
                grabberSubsystem.release(),
                telescopeSubsystem.moveToTargetAndHold(0.03));
    Command rightBack = followTrajectory("rightBack", new PathConstraints(1.72, 2.5));

    Command deliverUpperCone = deliverCone(Constants.ConePosition.TOP);
    Command deliverMiddleCone = deliverCone(Constants.ConePosition.MIDDLE);

    Command leftUpperConeAutonomous = deliverUpperCone.andThen(retrieveLeftBall);

    Command rightUpperConeAutonomous = deliverUpperCone.andThen(rightBack);

    Command middleAutonomous = deliverUpperCone.andThen(middleToChargingStation);

    Command middleAutonomousMiddleCone = deliverMiddleCone.andThen(middleToChargingStation);

    Command upperConeAutonomous =
        deliverUpperCone.andThen(elevatorSubsystem.lowerElevatorCommand());

    chooser.addOption("Charge Command", chargeCommand);
    chooser.addOption("Left Upper Cone Autonomous", leftUpperConeAutonomous);
    chooser.addOption("Right Upper Cone Autonomous", rightUpperConeAutonomous);
    chooser.addOption("Middle Autonomous Middle Cone", middleAutonomousMiddleCone);
    chooser.addOption("Middle Autonomous Upper Cone", middleAutonomous);
    chooser.addOption("Middle Autonomous No Cone", middleToChargingStation);
    chooser.addOption("Just Upper Cone Autonomous :(", upperConeAutonomous);
    chooser.setDefaultOption("Middle Autonomous", middleAutonomous);
    SmartDashboard.putData(chooser);

    inclineAngle.setDefaultDouble(0);
    inclineDirection.setDefaultDouble(0);
    pitch.setDefaultDouble(0);
    roll.setDefaultDouble(0);

    // Configure the button bindings
    configureButtonBindings();
    configureObjects();
    buildAutoEventMap();
  }

  public void configureObjects() {

    frontRightSpeedMotor.setInverted(false);
    frontLeftSpeedMotor.setInverted(true);
    backRightSpeedMotor.setInverted(true);
    backLeftSpeedMotor.setInverted(false);

    frontLeftWheel.getEncoder().setInverted(true);
    frontRightWheel.getEncoder().setInverted(true);
    backLeftWheel.getEncoder().setInverted(true);
    backRightWheel.getEncoder().setInverted(true);

    frontLeftAngleMotor.setInverted(false);
    frontRightAngleMotor.setInverted(false);
    backLeftAngleMotor.setInverted(false);
    backRightAngleMotor.setInverted(false);

    backRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
    backLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
    frontRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
    frontLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);

    elevatorMotor.setInverted(true);
    telescopeMotor.setInverted(true);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    telescopeMotor.setNeutralMode(NeutralMode.Brake);

    coastDrive();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick leftJoystick = new Joystick(0);
    Joystick rightJoystick = new Joystick(1);
    Joystick altJoystick = new Joystick(2);

    JoystickButton balanceChargingStationButton = new JoystickButton(leftJoystick, 1);
    JoystickButton turtleButton = new JoystickButton(rightJoystick, 1);
    JoystickButton fieldCentricButton = new JoystickButton(rightJoystick, 2);

    JoystickButton telescopeSubstation = new JoystickButton(altJoystick, 8);

    JoystickButton elevatorUpButton = new JoystickButton(altJoystick, 6);
    JoystickButton elevatorDownButton = new JoystickButton(altJoystick, 5);

    JoystickButton toggleGrabberButton = new JoystickButton(altJoystick, 9);

    Trigger pivotToTopPegButton = new Trigger(() -> altJoystick.getPOV() == 0);
    Trigger pivotToBottomButton = new Trigger(() -> altJoystick.getPOV() == 180);

    Trigger telescopeToOuterButton = new Trigger(() -> altJoystick.getPOV() == 90);
    Trigger telescopeToMiddleButton = new Trigger(() -> altJoystick.getPOV() == 270);
    JoystickButton telescopeToInButton = new JoystickButton(altJoystick, 7);

    JoystickButton alignRightOfAprilTagButton = new JoystickButton(leftJoystick, 4);
    JoystickButton alignAtAprilTagButton = new JoystickButton(leftJoystick, 2);
    JoystickButton alignLeftOfAprilTagButton = new JoystickButton(leftJoystick, 3);

    JoystickButton resetGyroButton = new JoystickButton(rightJoystick, 8);

    swerveDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              if (robot.isTeleopEnabled()) {
                swerveDrive.drive(
                    limitX.calculate(
                        applyDeadband(-leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND)
                            * DriverConstants.speedMultiplier),
                    limitY.calculate(
                        applyDeadband(-leftJoystick.getX(), DrivetrainConstants.DRIFT_DEADBAND)
                            * DriverConstants.speedMultiplier),
                    applyDeadband(
                        -rightJoystick.getX() * DriverConstants.angleMultiplier,
                        DrivetrainConstants.ROTATION_DEADBAND));
              } else {
                swerveDrive.drive(0, 0, 0);
              }
            },
            swerveDrive));

    fieldCentricButton.onTrue(swerveDrive.toggleFieldCentricCommand());

    balanceChargingStationButton.whileTrue(
        ledSubsystem
            .showBalancingColorCommand()
            .andThen(levelChargingStationCommand, ledSubsystem.showDockedColorCommand()));

    turtleButton.whileTrue(swerveDrive.turtleCommand());

    elevatorUpButton.whileTrue(elevatorSubsystem.raiseElevatorCommand());

    elevatorDownButton.whileTrue(elevatorSubsystem.lowerElevatorCommand());

    telescopeSubstation.onTrue(
        telescopeSubsystem.moveToTarget(ArmConstants.telescopeSubstationSetpoint));

    Command toggleGrabberWithLeds =
        grabberSubsystem
            .toggle()
            .andThen(
                Commands.either(
                    ledSubsystem.showGrabberOpenCommand(),
                    ledSubsystem.showGrabberClosedCommand(),
                    grabberSubsystem::isOpen));

    toggleGrabberButton.onTrue(toggleGrabberWithLeds);

    pivotToTopPegButton.onTrue(pivotSubsystem.moveToTarget(ArmConstants.topConePosition));
    pivotToBottomButton.onTrue(pivotSubsystem.moveToTarget(ArmConstants.bottomConePosition));

    telescopeToOuterButton.onTrue(
        telescopeSubsystem.moveToTarget(ArmConstants.telescopeOuterSetpoint));

    telescopeToMiddleButton.onTrue(
        telescopeSubsystem.moveToTarget(ArmConstants.telescopeMiddleSetpoint));

    telescopeToInButton.onTrue(telescopeSubsystem.moveToTarget(0.06));

    alignAtAprilTagButton.whileTrue(alignAtAprilTag);
    alignLeftOfAprilTagButton.whileTrue(alignLeftOfAprilTag);
    alignRightOfAprilTagButton.whileTrue(alignRightOfAprilTag);

    resetGyroButton.onTrue(
        new InstantCommand(
            () -> {
              odometry.zeroHeading();
              swerveDrive.resetLockRot();
              odometry.reset(
                  new Pose2d(
                      odometry.getPose().getX(),
                      odometry.getPose().getY(),
                      Rotation2d.fromDegrees(180)));
            },
            swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double applyDeadband(double val, double deadband) {
    if (Math.abs(val) < deadband) return 0;
    else return val;
  }

  public Odometry getOdometry() {
    return odometry;
  }

  public SwerveDriveSubsystem getSwerveDriveSubsystem() {
    return swerveDrive;
  }

  public void coastDrive() {
    frontLeftAngleMotor.setIdleMode(IdleMode.kCoast);
    frontRightAngleMotor.setIdleMode(IdleMode.kCoast);
    backLeftAngleMotor.setIdleMode(IdleMode.kCoast);
    backRightAngleMotor.setIdleMode(IdleMode.kCoast);

    frontLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
    frontRightSpeedMotor.setIdleMode(IdleMode.kCoast);
    backLeftSpeedMotor.setIdleMode(IdleMode.kCoast);
    backRightSpeedMotor.setIdleMode(IdleMode.kCoast);
  }

  public void periodic() {
    inclineAngle.setDouble(odometry.getInclineAngle().getDegrees());
    inclineDirection.setDouble(odometry.getInclineDirection().getDegrees());

    pitch.setDouble(odometry.getPitch().getDegrees());
    roll.setDouble(odometry.getRoll().getDegrees());
  }

  public void buildAutoEventMap() {

    eventMap.put(
        "pickUpBall",
        grabberSubsystem.grab().andThen(pivotSubsystem.moveToTargetUntilThere(0.995)));

    eventMap.put(
        "telescopeOut", telescopeSubsystem.moveToTarget(ArmConstants.telescopeOuterSetpoint));
  }

  private double getArmPositionForCone(Constants.ConePosition conePosition) {
    switch (conePosition) {
      case TOP:
        return ArmConstants.topConePosition;
      case MIDDLE:
        return ArmConstants.middleConePosition;
      case BOTTOM:
        return ArmConstants.bottomConePosition;
      default:
        return ArmConstants.substationConePosition;
    }
  }

  private double getTelescopePositionForCone(Constants.ConePosition conePosition) {
    switch (conePosition) {
      case TOP:
        return ArmConstants.telescopeOuterSetpoint;
      case MIDDLE:
        return ArmConstants.telescopeMiddleSetpoint;
      case BOTTOM:
        return ArmConstants.telescopeBottomSetpoint;
      default:
      case SUBSTATION:
        return ArmConstants.telescopeSubstationSetpoint;
    }
  }
}
