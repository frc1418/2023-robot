// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.commands.AlignByAprilTag;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LevelChargingStationCommand;
import frc.robot.common.Odometry;
import frc.robot.common.TrajectoryLoader;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    private final RobotBase robot;

    private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backRightSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainConstants.BACK_RIGHT_ENCODER);

    private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontRightSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_RIGHT_ENCODER);

    private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.BACK_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainConstants.BACK_LEFT_ENCODER);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontLeftSpeedMotor = new CANSparkMax(DrivetrainConstants.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainConstants.FRONT_LEFT_ENCODER);

    private WheelSubsystem backRightWheel = new WheelSubsystem (
        backRightAngleMotor, backRightSpeedMotor, backRightEncoder,
        DrivetrainConstants.BACK_RIGHT_LOC);
    public WheelSubsystem backLeftWheel = new WheelSubsystem (
      backLeftAngleMotor, backLeftSpeedMotor, backLeftEncoder,
      DrivetrainConstants.BACK_LEFT_LOC);
    private WheelSubsystem frontRightWheel = new WheelSubsystem (
      frontRightAngleMotor, frontRightSpeedMotor, frontRightEncoder,
      DrivetrainConstants.FRONT_RIGHT_LOC);
    private WheelSubsystem frontLeftWheel = new WheelSubsystem (
      frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftEncoder,
      DrivetrainConstants.FRONT_LEFT_LOC);
    

    AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable table = ntInstance.getTable("/components/Odometry");

    private final NetworkTableEntry inclineAngle = table.getEntry("inclineAngle");
    private final NetworkTableEntry inclineDirection = table.getEntry("inclineDirection");

    private final NetworkTableEntry pitch = table.getEntry("pitch");
    private final NetworkTableEntry roll = table.getEntry("roll");

    private SwerveModulePosition[] positions = new SwerveModulePosition[] {
      frontLeftWheel.getSwerveModulePosition(),
      frontRightWheel.getSwerveModulePosition(),
      backLeftWheel.getSwerveModulePosition(),
      backRightWheel.getSwerveModulePosition()
    };

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, gyro.getRotation2d(), positions);

    private LimelightSubsystem limelight = new LimelightSubsystem();
    private Odometry odometry = new Odometry(gyro, driveOdometry, positions, limelight);
    
    private SwerveDriveSubsystem swerveDrive = new SwerveDriveSubsystem(
        backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
        DrivetrainConstants.SWERVE_KINEMATICS, odometry);


    private CANSparkMax pivotMotor = new CANSparkMax(ArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private TalonFX telescopeMotor = new TalonFX(ArmConstants.TELESCOPE_MOTOR_ID);
    private ArmSubsystem armSubsystem = new ArmSubsystem(pivotMotor, telescopeMotor);

    private DoubleSolenoid leftSolenoid = new DoubleSolenoid(GrabberConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, GrabberConstants.LEFT_SOLENOID_FORWARD, GrabberConstants.LEFT_SOLENOID_REVERSE);
    private DoubleSolenoid rightSolenoid = new DoubleSolenoid(GrabberConstants.PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, GrabberConstants.RIGHT_SOLENOID_FORWARD, GrabberConstants.RIGHT_SOLENOID_REVERSE);
    private GrabberSubsystem grabberSubsystem = new GrabberSubsystem(leftSolenoid, rightSolenoid);

    private TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR_ID);
    private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorMotor);

    // LOAD TRAJECTORIES
    private final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
    private final HashMap<String, Trajectory> trajectories = trajectoryLoader.loadTrajectories();

    // SENDABLE CHOOSER
    private final SendableChooser<Command> chooser = new SendableChooser<>();
    // private final Command chargeCommand = new ChargeCommand(swerveDrive, odometry, trajectories);

    private final LevelChargingStationCommand levelChargingStationCommand = new LevelChargingStationCommand(odometry, swerveDrive);
    private final AlignByAprilTag alignAtAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, 0, -1);//1.1);
    private final AlignByAprilTag alignLeftOfAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.58, -1);
    private final AlignByAprilTag alignRightOfAprilTag = new AlignByAprilTag(swerveDrive, limelight, odometry, -0.58, -1);
    private final AlignByAprilTag alignRightSubstation = new AlignByAprilTag(swerveDrive, limelight, odometry, 0.584, -1);
    private final AlignByAprilTag alignLeftSubstation = new AlignByAprilTag(swerveDrive, limelight, odometry, -0.584, -1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotBase robot) {
      this.robot = robot;

      // chooser.setDefaultOption("Charge Command", chargeCommand);
      // chooser.addOption("Command", chargeCommand);
      SmartDashboard.putData(chooser);

      inclineAngle.setDefaultDouble(0);
      inclineDirection.setDefaultDouble(0);
      pitch.setDefaultDouble(0);
      roll.setDefaultDouble(0);

      // Configure the button bindings
      configureButtonBindings();
      configureObjects();
    }

    public void configureObjects() {
      // frontLeftAngleMotor.setInverted(true);
      // backLeftAngleMotor.setInverted(true);

      // frontLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      // frontRightAngleMotor.setIdleMode(IdleMode.kBrake);
      // backLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      // backRightAngleMotor.setIdleMode(IdleMode.kBrake);

      // frontLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      // frontRightSpeedMotor.setIdleMode(IdleMode.kBrake);
      // backLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      // backRightSpeedMotor.setIdleMode(IdleMode.kBrake);

      // frontLeftAngleMotor.setInverted(true);
      // frontRightAngleMotor.setInverted(true);
      // backLeftAngleMotor.setInverted(true);

      frontRightSpeedMotor.setInverted(true);

      frontLeftWheel.getEncoder().setInverted(true);
      frontRightWheel.getEncoder().setInverted(true);
      backLeftWheel.getEncoder().setInverted(true);
      backRightWheel.getEncoder().setInverted(true);


      frontLeftAngleMotor.setInverted(false);
      frontRightAngleMotor.setInverted(false);
      backLeftAngleMotor.setInverted(false);
      backRightAngleMotor.setInverted(false);

      backRightSpeedMotor.setInverted(false);

      backRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
      backLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
      frontRightWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
      frontLeftWheel.getEncoder().setZeroOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);
      // frontRightSpeedMotor.setInverted(true);
      // backRightSpeedMotor.setInverted(true);

      // backRightEncoder.setPositionOffset(DrivetrainConstants.BACK_RIGHT_ENCODER_OFFSET);
      // backLeftEncoder.setPositionOffset(DrivetrainConstants.BACK_LEFT_ENCODER_OFFSET);
      // frontRightEncoder.setPositionOffset(DrivetrainConstants.FRONT_RIGHT_ENCODER_OFFSET);
      // frontLeftEncoder.setPositionOffset(DrivetrainConstants.FRONT_LEFT_ENCODER_OFFSET);

      elevatorMotor.setInverted(true);
      telescopeMotor.setInverted(true);
      pivotMotor.setIdleMode(IdleMode.kBrake);
      elevatorMotor.setNeutralMode(NeutralMode.Brake);
      telescopeMotor.setNeutralMode(NeutralMode.Brake);
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

      JoystickButton pivotUpButton = new JoystickButton(altJoystick, 4);
      JoystickButton pivotDownButton = new JoystickButton(altJoystick, 1);

      JoystickButton telescopeOutButton = new JoystickButton(altJoystick, 2);
      JoystickButton telescopeInButton = new JoystickButton(altJoystick, 3);

      JoystickButton elevatorUpButton = new JoystickButton(altJoystick, 6);
      JoystickButton elevatorDownButton = new JoystickButton(altJoystick, 5);

      JoystickButton toggleGrabberButton = new JoystickButton(altJoystick, 9);

      Trigger pivotToTopPegButton = new Trigger(() -> altJoystick.getPOV() == 0);
      Trigger pivotToSubstationButton = new Trigger(() -> altJoystick.getPOV() == 45);
      Trigger pivotToBottomButton = new Trigger(() -> altJoystick.getPOV() == 180);

      Trigger telescopeToOuterButton = new Trigger(() -> altJoystick.getPOV() == 90);
      Trigger telescopeToMiddleButton = new Trigger(() -> altJoystick.getPOV() == 270);
      JoystickButton telescopeToInButton = new JoystickButton(altJoystick, 7);

      Trigger elevatorToMiddleButton = new Trigger(() -> altJoystick.getPOV() == 45);
      JoystickButton alignRightOfAprilTagButton = new JoystickButton(leftJoystick, 4);
      JoystickButton alignAtAprilTagButton = new JoystickButton(leftJoystick, 2);
      JoystickButton alignLeftOfAprilTagButton = new JoystickButton(leftJoystick, 3);
      JoystickButton alignLeftSubstationButton = new JoystickButton(leftJoystick, 3);
      JoystickButton alignRightSubstationButton = new JoystickButton(altJoystick, 4);


      swerveDrive.setDefaultCommand(new RunCommand(
          () -> {
            if (robot.isTeleopEnabled()) {
              swerveDrive.drive(
                  applyDeadband(leftJoystick.getY(), DrivetrainConstants.DRIFT_DEADBAND) * DriverConstants.speedMultiplier,
                  applyDeadband(leftJoystick.getX(),DrivetrainConstants.DRIFT_DEADBAND) * DriverConstants.speedMultiplier,
                  applyDeadband(rightJoystick.getX() * DriverConstants.angleMultiplier, DrivetrainConstants.ROTATION_DEADBAND));
            } else {
              swerveDrive.drive(0, 0, 0);
            }
          },
          swerveDrive));

      fieldCentricButton.onTrue(new InstantCommand(
          () -> {
            System.out.println("FIELD CENTRIC TOGGLED");
            swerveDrive.toggleFieldCentric();
          }, swerveDrive));
      
      balanceChargingStationButton.whileTrue(levelChargingStationCommand);

      turtleButton.whileTrue(new RunCommand(() -> swerveDrive.turtle(), swerveDrive));

      pivotUpButton.whileTrue(new RunCommand(() -> armSubsystem.setPivotMotor(0.2), armSubsystem));
      pivotUpButton.onFalse(new InstantCommand(() -> armSubsystem.setPivotMotor(0), armSubsystem));

      pivotDownButton.whileTrue(new RunCommand(() -> armSubsystem.setPivotMotorVoltage(-0.1), armSubsystem));
      pivotDownButton.onFalse(new InstantCommand(() -> armSubsystem.setPivotMotor(0), armSubsystem));

      elevatorUpButton.whileTrue(new RunCommand(() -> elevatorSubsystem.setElevatorMotor(0.9), elevatorSubsystem));
      elevatorUpButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setElevatorMotor(0), elevatorSubsystem));

      elevatorDownButton.whileTrue(new RunCommand(() -> {
        elevatorSubsystem.setElevatorMotor(-0.9);
      }, elevatorSubsystem));
      elevatorDownButton.onFalse(new InstantCommand(() -> elevatorSubsystem.setElevatorMotor(0), elevatorSubsystem));

      telescopeOutButton.whileTrue(new RunCommand(() -> {
        armSubsystem.setTelescopeMotor(0.2);
      }, armSubsystem));
      telescopeOutButton.onFalse(new InstantCommand(() -> armSubsystem.setTelescopeMotor(0), armSubsystem));

      telescopeInButton.whileTrue(new RunCommand(() -> armSubsystem.setTelescopeMotor(-0.2), armSubsystem));
      telescopeInButton.onFalse(new InstantCommand(() -> armSubsystem.setTelescopeMotor(0), armSubsystem));

      toggleGrabberButton.onTrue(new InstantCommand(() -> grabberSubsystem.toggle(), grabberSubsystem));

      pivotToTopPegButton.onTrue(new RunCommand(() -> armSubsystem.setPivotPosition(0.01), armSubsystem));
      pivotToSubstationButton.onTrue(new RunCommand(() -> armSubsystem.setPivotPosition(0), armSubsystem));
      pivotToBottomButton.onTrue(new RunCommand(() -> armSubsystem.setPivotPosition(0.88), armSubsystem));

      telescopeToOuterButton.onTrue(new RunCommand(() -> {
        armSubsystem.setTelescopePosition(ArmConstants.telescopeOuterSetpoint);
      }, armSubsystem));

      telescopeToMiddleButton.onTrue(new RunCommand(() -> {
        armSubsystem.setTelescopePosition(0.25);
      }, armSubsystem));

      telescopeToInButton.onTrue(new RunCommand(() -> {
        armSubsystem.setTelescopePosition(0.03);
      }, armSubsystem));

      elevatorToMiddleButton.onTrue(new RunCommand(() -> elevatorSubsystem.setElevatorHeight(0), elevatorSubsystem));


      armSubsystem.setDefaultCommand(new RunCommand(() -> {
        armSubsystem.setPivotPosition(armSubsystem.getPivotPosition());
      }, armSubsystem));

      alignAtAprilTagButton.whileTrue(alignAtAprilTag);
      alignLeftOfAprilTagButton.whileTrue(alignLeftOfAprilTag);
      alignRightOfAprilTagButton.whileTrue(alignRightOfAprilTag);
      alignLeftSubstationButton.whileTrue(alignLeftSubstation);
      alignRightSubstationButton.whileTrue(alignRightSubstation);
      
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      // odometry.zeroHeading();
      return null;
    }

    public double applyDeadband(double val, double deadband){
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
}
