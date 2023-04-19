package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.common.Odometry
import frc.robot.subsystems.SwerveDriveSubsystem

class LevelChargingStationCommand(odometry: Odometry, swerveDriveSubsystem: SwerveDriveSubsystem) :
    PIDCommand(
        PIDController(.0026, 0.0, .008),
        odometry.getInclineAngle()::getDegrees,
        0.0,
        { x: Double -> swerveDriveSubsystem.strafe(odometry.inclineDirection, -x) },
        swerveDriveSubsystem
    ) {
    init {
        controller.setTolerance(.05, .05)
    }

    override fun isFinished(): Boolean {
        return controller.atSetpoint()
    }
}
