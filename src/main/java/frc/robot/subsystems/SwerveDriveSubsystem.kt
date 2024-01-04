package frc.robot.subsystems

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.common.Odometry

class SwerveDriveSubsystem(
    private val backRight: WheelSubsystem,
    private val backLeft: WheelSubsystem,
    private val frontRight: WheelSubsystem,
    private val frontLeft: WheelSubsystem,
    private val kinematics: SwerveDriveKinematics,
    private val odometry: Odometry
) : SubsystemBase() {
    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/drivetrain")
    private val ntIsFieldCentric = table.getEntry("isFieldCentric")
    private val ntVelocity = table.getEntry("wheelvelocity")
    private val odometryTable = ntInstance.getTable("/common/Odometry")
    private val ntOdometryPose = odometryTable.getEntry("odometryPose")
    private val rotationController = PIDController(0.04, 0.0, 0.0)
    var fieldCentric = true
    private var lockedRot: Double

    init {
        ntIsFieldCentric.setBoolean(fieldCentric)
        lockedRot = odometry.heading
    }

    fun drive(x: Double, y: Double, rot: Double, fieldCentric_: Boolean = fieldCentric) {
        var rotOut = rot
        if (rot == 0.0) {
            rotOut = rotationController.calculate(odometry.heading, lockedRot)
        } else {
            lockedRot = odometry.heading
        }
        var speeds = ChassisSpeeds(x, y, rotOut)
        if (fieldCentric_) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, odometry.rotation2d)
        }
        drive(speeds)
    }

    fun drive(speeds: ChassisSpeeds) {
        // Convert to module states
        val moduleStates = kinematics.toSwerveModuleStates(speeds)
        drive(moduleStates)
    }

    private fun drive(moduleStates: Array<SwerveModuleState>) {
        val frontLeftState = moduleStates[0]
        val frontRightState = moduleStates[1]
        val backLeftState = moduleStates[2]
        val backRightState = moduleStates[3]
        ntVelocity.setDouble(frontLeftState.speedMetersPerSecond)
        frontLeft.drive(frontLeftState)
        frontRight.drive(frontRightState)
        backLeft.drive(backLeftState)
        backRight.drive(backRightState)
    }

    fun strafe(direction: Rotation2d, speed: Double) {
        drive(speed * direction.cos, speed * direction.sin, 0.0)
    }

    val turtleCommand: Command
        get() = run { turtle() }

    private fun turtle() {
        val negative45 = SwerveModuleState(0.0, Rotation2d(-Math.PI / 4))
        val positive45 = SwerveModuleState(0.0, Rotation2d(Math.PI / 4))
        val moduleStates = arrayOf(negative45, positive45, positive45, negative45)
        drive(moduleStates)
    }

    override fun periodic() {
        odometry.update(positions)
        ntOdometryPose.setString(odometry.pose.toString())
        frontLeft.periodic()
        frontRight.periodic()
        backLeft.periodic()
        backRight.periodic()
        if (DriverStation.isAutonomousEnabled()) {
            lockedRot = odometry.heading
        }
        ntIsFieldCentric.setBoolean(fieldCentric)
    }

    private val positions: Array<SwerveModulePosition>
        get() =
            arrayOf(
                frontLeft.swerveModulePosition,
                frontRight.swerveModulePosition,
                backLeft.swerveModulePosition,
                backRight.swerveModulePosition
            )

    val disableFieldCentricCommand: CommandBase
        get() = runOnce { fieldCentric = false }

    fun toggleFieldCentric(): CommandBase = runOnce { fieldCentric = !fieldCentric }

    fun stop(): Command = runOnce { drive(0.0, 0.0, 0.0) }

    fun resetLockRot() {
        lockedRot = odometry.heading
    }
}
