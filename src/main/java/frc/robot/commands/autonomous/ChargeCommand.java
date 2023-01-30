// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainSubsystem;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class ChargeCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "chargeLeft";

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChargeCommand(SwerveDriveSubsystem swerveDriveSubsystem, Odometry odometry, HashMap<String, Trajectory> trajectories) {

    PathPlannerTrajectory charge = PathPlanner.loadPath("chargeLeft", new PathConstraints(0.4, 0.4));
    
    // TrajectoryConfig config = new TrajectoryConfig(0.4, 0.4)
    //       .setKinematics(DrivetrainSubsystem.swerveKinematics);

    // Trajectory traj = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
    //   List.of(new Translation2d(0, 1)),
    //   new Pose2d(0, 2, Rotation2d.fromDegrees(90)),
    //   config
    // );

    odometry.zeroHeading();
    odometry.reset(charge.getInitialHolonomicPose());

    // PIDController speedControllerX = new PIDController(1, 0, 0.00);
    // PIDController speedControllerY = new PIDController(1, 0, 0.000);
    PIDController angleController = new PIDController(0.1, 0, 0);

    // angleController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        charge,
        odometry::getPose,
        DrivetrainSubsystem.swerveKinematics,
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        angleController,
        swerveDriveSubsystem::drive,
        swerveDriveSubsystem);

    // Rotation2d endingRotation = Rotation2d.fromDegrees(90);//traj.sample(traj.getTotalTimeSeconds()).poseMeters.getRotation();
    
    addCommands(
      new InstantCommand(() -> odometry.reset(charge.getInitialHolonomicPose())),
      swerveControllerCommand
    );


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }
}
