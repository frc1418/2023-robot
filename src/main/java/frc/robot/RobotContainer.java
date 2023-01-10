// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DrivetrainSubsystem;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriverSubsystem;
import frc.robot.subsystems.WheelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

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

    private WheelSubsystem backRightWheel = new WheelSubsystem (
        DrivetrainSubsystem.BACK_RIGHT_ANGLE_ID, DrivetrainSubsystem.BACK_RIGHT_SPEED_ID,
        0, DrivetrainSubsystem.m_backRightLocation);
    private WheelSubsystem backLeftWheel = new WheelSubsystem (
        DrivetrainSubsystem.BACK_LEFT_ANGLE_ID, DrivetrainSubsystem.BACK_LEFT_SPEED_ID,
        0, DrivetrainSubsystem.m_backLeftLocation);
    private WheelSubsystem frontRightWheel = new WheelSubsystem (
        DrivetrainSubsystem.FRONT_RIGHT_ANGLE_ID, DrivetrainSubsystem.FRONT_RIGHT_SPEED_ID,
        0, DrivetrainSubsystem.m_frontRightLocation);
    private WheelSubsystem frontLeftWheel = new WheelSubsystem (
        DrivetrainSubsystem.FRONT_LEFT_ANGLE_ID, DrivetrainSubsystem.FRONT_LEFT_SPEED_ID,
        0, DrivetrainSubsystem.m_frontLeftLocation);
    
    private SwerveDriverSubsystem swerveDrive = new SwerveDriverSubsystem(backRightWheel, backLeftWheel, frontRightWheel, frontLeftWheel);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(RobotBase robot) {
      this.robot = robot;

      // Configure the button bindings
      configureButtonBindings();
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

      swerveDrive.setDefaultCommand(new RunCommand(
          () -> {
            if (robot.isTeleopEnabled()) {
              swerveDrive.drive(leftJoystick.getY(), leftJoystick.getX(), rightJoystick.getX());
            } else {
              swerveDrive.drive(0, 0, 0);
            }
          },
          swerveDrive));

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
}
