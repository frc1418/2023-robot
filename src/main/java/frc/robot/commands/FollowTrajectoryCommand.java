// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.common.Odometry;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class FollowTrajectoryCommand extends CommandBase {
    private Trajectory trajectory;
    private Odometry odometry;
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private boolean resetOdometry;
    private HolonomicDriveController controller;

    private long startingTime;
    private Rotation2d desiredRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FollowTrajectoryCommand(
        Trajectory trajectory,
        Odometry odometry,
        SwerveDriveSubsystem swerveDriveSubsystem,
        boolean resetOdometry,
        Rotation2d desiredRotation) {

    this.trajectory = trajectory;
    this.odometry = odometry;
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.resetOdometry = resetOdometry;
    this.desiredRotation = desiredRotation;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDriveSubsystem);

    PIDController speedControllerX = new PIDController(1.5, 0, 0.00);
    PIDController speedControllerY = new PIDController(1.5, 0, 0.000);
    ProfiledPIDController angleController = new ProfiledPIDController(0.02, 0, 0,
        new TrapezoidProfile.Constraints(2*Math.PI, Math.PI));

    controller = new HolonomicDriveController(speedControllerX, speedControllerY, angleController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingTime = System.nanoTime();
    if (resetOdometry){
      odometry.reset(trajectory.getInitialPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectory.sample(getCurrentTime());
    ChassisSpeeds adjustedSpeeds = controller.calculate(odometry.getPose(), goal, desiredRotation);
    System.out.println(adjustedSpeeds.toString());
    System.out.println(goal.toString());
    System.out.println(odometry.getPose());
    System.out.println("");
    swerveDriveSubsystem.drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getCurrentTime() {
    return (System.nanoTime() - startingTime) / (double) 1000000000;
  }
}
