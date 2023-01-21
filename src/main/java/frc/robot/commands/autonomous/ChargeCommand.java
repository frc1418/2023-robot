// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ChargeCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "charge";

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChargeCommand(SwerveDriveSubsystem swerveDriveSubsystem, Odometry odometry, HashMap<String, Trajectory> trajectories) {

    Trajectory charge = trajectories.get(TRAJECTORY_NAME);

    Rotation2d endingRotation = charge.sample(charge.getTotalTimeSeconds()).poseMeters.getRotation();

    addCommands(
        new FollowTrajectoryCommand(charge, odometry, swerveDriveSubsystem, true, endingRotation)
    );


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }
}
