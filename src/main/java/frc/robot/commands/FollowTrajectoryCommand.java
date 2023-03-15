// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.common.Odometry;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FollowTrajectoryCommand extends SequentialCommandGroup {
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTrajectoryCommand(
        String trajectoryName,
        Odometry odometry,
        SwerveDriveSubsystem swerveDriveSubsystem, HashMap<String, Command> eventMap, PathConstraints pathConstraints) {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryName, pathConstraints);

    PIDController speedControllerX = new PIDController(0.001, 0, 0);
    PIDController speedControllerY = new PIDController(0.001,0,0);//1.3, 0, 0.000);
    PIDController angleController = new PIDController(0.9,0,0);//1.8, 0, 0);

    PIDConstants angleConstants = new PIDConstants(0.9, 0, 0);
    PIDConstants translationConstants = new PIDConstants(0.001, 0, 0);


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(odometry::getPose,
      odometry::reset,
      translationConstants,
      angleConstants,
      swerveDriveSubsystem::drive,
      eventMap,
      false,
      swerveDriveSubsystem
      );

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        odometry::getPose,
        DrivetrainConstants.SWERVE_KINEMATICS,
        speedControllerX,
        speedControllerY,
        angleController,
        swerveDriveSubsystem::drive,
        false,
        swerveDriveSubsystem);

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);

      addCommands(
        new InstantCommand(() -> odometry.reset(trajectory.getInitialHolonomicPose())),
        autoBuilder.fullAuto(trajectory),
        new PrintCommand("DONE"),
        new InstantCommand(() -> swerveDriveSubsystem.drive(0, 0, 0))
      );
    }
}
