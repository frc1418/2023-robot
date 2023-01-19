// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivetrainSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriverSubsystem;
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

    private CANSparkMax backRightAngleMotor = new CANSparkMax(DrivetrainSubsystem.BACK_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backRightSpeedMotor = new CANSparkMax(DrivetrainSubsystem.BACK_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backRightEncoder = new AnalogEncoder(DrivetrainSubsystem.BACK_RIGHT_ENCODER);

    private CANSparkMax frontRightAngleMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_RIGHT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontRightSpeedMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_RIGHT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontRightEncoder = new AnalogEncoder(DrivetrainSubsystem.FRONT_RIGHT_ENCODER);

    private CANSparkMax backLeftAngleMotor = new CANSparkMax(DrivetrainSubsystem.BACK_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax backLeftSpeedMotor = new CANSparkMax(DrivetrainSubsystem.BACK_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder backLeftEncoder = new AnalogEncoder(DrivetrainSubsystem.BACK_LEFT_ENCODER);

    private CANSparkMax frontLeftAngleMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_LEFT_ANGLE_ID, MotorType.kBrushless);
    private CANSparkMax frontLeftSpeedMotor = new CANSparkMax(DrivetrainSubsystem.FRONT_LEFT_SPEED_ID, MotorType.kBrushless);
    private AnalogEncoder frontLeftEncoder = new AnalogEncoder(DrivetrainSubsystem.FRONT_LEFT_ENCODER);

    private WheelSubsystem backRightWheel = new WheelSubsystem (
        backRightAngleMotor, backRightSpeedMotor, backRightEncoder,
        DrivetrainSubsystem.m_backRightLocation);
    public WheelSubsystem backLeftWheel = new WheelSubsystem (
      backLeftAngleMotor, backLeftSpeedMotor, backLeftEncoder,
      DrivetrainSubsystem.m_backLeftLocation);
    private WheelSubsystem frontRightWheel = new WheelSubsystem (
      frontRightAngleMotor, frontRightSpeedMotor, frontRightEncoder,
      DrivetrainSubsystem.m_frontRightLocation);
    private WheelSubsystem frontLeftWheel = new WheelSubsystem (
      frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftEncoder,
      DrivetrainSubsystem.m_frontLeftLocation);
    

    Gyro gyro = new AHRS(SPI.Port.kMXP);

    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          frontLeftWheel.getLocation(),
          frontRightWheel.getLocation(),
          backLeftWheel.getLocation(),
          backRightWheel.getLocation());

    private SwerveModulePosition[] positions = new SwerveModulePosition[] {
      frontLeftWheel.getSwerveModulePosition(),
      frontRightWheel.getSwerveModulePosition(),
      backLeftWheel.getSwerveModulePosition(),
      backRightWheel.getSwerveModulePosition()
    };

    private SwerveDriveOdometry driveOdometry = new SwerveDriveOdometry(swerveKinematics, gyro.getRotation2d(), positions);

    private Odometry odometry = new Odometry(gyro, driveOdometry, positions);
    
    private SwerveDriverSubsystem swerveDrive = new SwerveDriverSubsystem(
        backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel,
        swerveKinematics, odometry);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotBase robot) {
      this.robot = robot;

      // Configure the button bindings
      configureButtonBindings();
    }

    public void configureObjects() {
      // frontLeftAngleMotor.setInverted(true);
      // backLeftAngleMotor.setInverted(true);

      frontLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      frontRightAngleMotor.setIdleMode(IdleMode.kBrake);
      backLeftAngleMotor.setIdleMode(IdleMode.kBrake);
      backRightAngleMotor.setIdleMode(IdleMode.kBrake);

      frontLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      frontRightSpeedMotor.setIdleMode(IdleMode.kBrake);
      backLeftSpeedMotor.setIdleMode(IdleMode.kBrake);
      backRightSpeedMotor.setIdleMode(IdleMode.kBrake);

      frontLeftAngleMotor.setInverted(true);
      frontRightAngleMotor.setInverted(true);
      backLeftAngleMotor.setInverted(true);
      backRightAngleMotor.setInverted(true);

      frontRightSpeedMotor.setInverted(true);
      backRightSpeedMotor.setInverted(true);

      backRightEncoder.setPositionOffset(DrivetrainSubsystem.BACK_RIGHT_ENCODER_OFFSET);
      backLeftEncoder.setPositionOffset(DrivetrainSubsystem.BACK_LEFT_ENCODER_OFFSET);
      frontRightEncoder.setPositionOffset(DrivetrainSubsystem.FRONT_RIGHT_ENCODER_OFFSET);
      frontLeftEncoder.setPositionOffset(DrivetrainSubsystem.FRONT_LEFT_ENCODER_OFFSET);
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

      

      JoystickButton resetEncoderButton = new JoystickButton(rightJoystick, 3);

      JoystickButton fieldCentricButton = new JoystickButton(leftJoystick, 2);


      swerveDrive.setDefaultCommand(new RunCommand(
          () -> {
            if (robot.isTeleopEnabled()) {
              swerveDrive.drive(
                  applyDeadband(-leftJoystick.getY() / 2, DrivetrainSubsystem.DRIFT_DEADBAND),
                  applyDeadband(leftJoystick.getX() / 2,DrivetrainSubsystem.DRIFT_DEADBAND),
                  applyDeadband(-rightJoystick.getX() / 32, DrivetrainSubsystem.ROTATION_DEADBAND));
            } else {
              swerveDrive.drive(0, 0, 0);
            }
          },
          swerveDrive));


      fieldCentricButton.onTrue(new InstantCommand(
          () -> {
            swerveDrive.toggleFieldCentric();
          }));


      fieldCentricButton.onTrue(new InstantCommand(
          () -> {
            System.out.println("FIELD CENTRIC TOGGLED");
            swerveDrive.toggleFieldCentric();
          }));

      // resetEncoderButton.whileTrue(new RunCommand(
      //   () -> {
      //     System.out.println("RESET");
      //     swerveDrive.resetEncoders();
      // }, swerveDrive));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An ExampleCommand will run in autonomous
      return m_autoCommand;
    }

    public double applyDeadband(double val, double deadband){
      if (Math.abs(val) < deadband) return 0;
      else return val;
    }

    public Odometry getOdometry() {
      return odometry;
    }
}
