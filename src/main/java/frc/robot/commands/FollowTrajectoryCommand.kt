// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.common.Odometry
import frc.robot.subsystems.SwerveDriveSubsystem

/** An example command that uses an example subsystem. */
class FollowTrajectoryCommand(
    trajectoryName: String?,
    odometry: Odometry,
    swerveDriveSubsystem: SwerveDriveSubsystem,
    eventMap: HashMap<String, Command?>?,
    pathConstraints: PathConstraints?
) : SequentialCommandGroup() {
    init {
        val trajectory = PathPlanner.loadPath(trajectoryName, pathConstraints)
        val angleConstants = PIDConstants(0.9, 0.0, 0.0)
        val translationConstants = PIDConstants(0.001, 0.0, 0.0)
        val autoBuilder =
            SwerveAutoBuilder(
                odometry::pose,
                odometry::reset,
                translationConstants,
                angleConstants,
                swerveDriveSubsystem::drive,
                eventMap,
                true,
                swerveDriveSubsystem
            )
        addCommands(
            PrintCommand(trajectoryName),
            Commands.run({ odometry.reset(trajectory.initialHolonomicPose) }),
            autoBuilder.fullAuto(trajectory),
            PrintCommand("DONE"),
            swerveDriveSubsystem.stop
        )
    }
}
