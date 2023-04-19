package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants.LimelightDirections
import frc.robot.common.Odometry
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveDriveSubsystem
import kotlin.math.atan2
import kotlin.math.hypot

class AlignByAprilTag(
    private val swerveDrive: SwerveDriveSubsystem,
    private val limelight: LimelightSubsystem,
    private val odometry: Odometry,
    private val targetX: Double,
    private val targetY: Double
) : CommandBase() {
    private val speedRotController: PIDController = PIDController(0.05, 0.0, 0.0)
    private val speedController: ProfiledPIDController =
        ProfiledPIDController(1.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0))

    init {
        speedRotController.enableContinuousInput(-180.0, 180.0)
        addRequirements(swerveDrive, limelight)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val robotPose =
            Pose2d(Translation2d(odometry.pose.y, odometry.pose.x), odometry.pose.rotation)
        val targetAngle = limelight.targetRotation.angle
        val targetPose: Pose2d =
            when (limelight.targetRotation) {
                LimelightDirections.GRID_SIDE ->
                    Pose2d(Translation2d(targetX, -targetY), Rotation2d.fromDegrees(targetAngle))
                LimelightDirections.SUBSTATION_SIDE ->
                    Pose2d(
                        Translation2d(targetX * 1.45, targetY * 1.3),
                        Rotation2d.fromDegrees(targetAngle)
                    )
            }

        var x: Double
        var y: Double
        val dx = robotPose.x - targetPose.x
        val dy = robotPose.y - targetPose.y
        val distance = hypot(dx, dy)
        val angleToTarget = atan2(dx, dy) * 180 / Math.PI
        val rot: Double = speedRotController.calculate(odometry.pose.rotation.degrees, targetAngle)
        var speed = -speedController.calculate(distance, 0.0)

        // System.out.println("DISTANCE: " + distance);
        val direction = Rotation2d.fromDegrees(180 + angleToTarget - odometry.heading + targetAngle)
        x = direction.cos * speed
        y = direction.sin * speed
        if (limelight.targetRotation == LimelightDirections.GRID_SIDE) {
            y *= -1.0
            x *= -1.0
        }
        swerveDrive.drive(x, y, rot, false)
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        println("END")
        swerveDrive.drive(0.0, 0.0, 0.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
