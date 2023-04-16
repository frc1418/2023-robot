package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LevelChargingStationCommand extends PIDCommand {

  public LevelChargingStationCommand(Odometry odometry, SwerveDriveSubsystem swerveDriveSubsystem) {

    super(
        new PIDController(.0026, 0, .008),
        () -> odometry.getInclineAngle().getDegrees(),
        0,
        (x) -> swerveDriveSubsystem.strafe(odometry.getInclineDirection(), -x),
        swerveDriveSubsystem);

    getController().setTolerance(.05, .05);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
