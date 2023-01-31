// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainSubsystem;
import frc.robot.common.Odometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FollowTrajectoryCommand extends SequentialCommandGroup {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTrajectoryCommand(
        PathPlannerTrajectory trajectory,
        Odometry odometry,
        SwerveDriveSubsystem swerveDriveSubsystem) {

    odometry.zeroHeading();

    PIDController speedControllerX = new PIDController(1.5, 0, 0.00);
    PIDController speedControllerY = new PIDController(1.5, 0, 0.000);
    PIDController angleController = new PIDController(0, 0, 0);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        odometry::getPose,
        DrivetrainSubsystem.swerveKinematics,
        speedControllerX,
        speedControllerY,
        angleController,
        swerveDriveSubsystem::drive,
        swerveDriveSubsystem);

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);

    addCommands(
      new InstantCommand(() -> odometry.reset(trajectory.getInitialHolonomicPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveDriveSubsystem.drive(0, 0, 0))
    );
    }
}
