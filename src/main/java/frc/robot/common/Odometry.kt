package frc.robot.common

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants.LimelightDirections
import frc.robot.subsystems.LimelightSubsystem
import kotlin.math.*

class Odometry(
    private val gyro: AHRS,
    private val odometry: SwerveDriveOdometry,
    private var modulePositions: Array<SwerveModulePosition>,
    private val limelight: LimelightSubsystem
) {
    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/Odometry")
    private val inclineAngle = table.getEntry("inclineAngle")
    private val inclineDirectionNt = table.getEntry("inclineDirection")
    private val pitchNt = table.getEntry("pitch")
    private val rollNt = table.getEntry("roll")

    init {
        inclineAngle.setDefaultDouble(0.0)
        inclineDirectionNt.setDefaultDouble(0.0)
        pitchNt.setDefaultDouble(0.0)
        rollNt.setDefaultDouble(0.0)
    }

    fun update(newPositions: Array<SwerveModulePosition>) {
        // Get the rotation of the robot from the gyro.
        if (limelight.isDetecting && DriverStation.isTeleop()) {
            if (limelight.targetRotation == LimelightDirections.GRID_SIDE)
                reset(
                    Pose2d(Translation2d(-limelight.yDistance, limelight.xDistance), pose.rotation)
                )
            else
                reset(
                    Pose2d(Translation2d(limelight.yDistance, -limelight.xDistance), pose.rotation)
                )
        }
        modulePositions = newPositions
        inclineAngle.setDouble(getInclineAngle().degrees)
        inclineDirectionNt.setDouble(inclineDirection.degrees)
        pitchNt.setDouble(pitch.degrees)
        rollNt.setDouble(roll.degrees)
    }

    fun reset(pose: Pose2d) {
        odometry.resetPosition(gyro.rotation2d, modulePositions, pose)
    }

    fun zeroHeading() {
        gyro.reset()
    }

    fun setAngleOffset(offset: Double) {
        gyro.angleAdjustment = offset
    }

    val pose: Pose2d
        get() = odometry.poseMeters
    val heading: Double
        get() = gyro.rotation2d.degrees
    val rotation2d: Rotation2d
        get() = gyro.rotation2d

    private val pitch: Rotation2d
        get() =
            // switched because of roborio placement
            Rotation2d.fromDegrees(gyro.roll.toDouble())

    private val roll: Rotation2d
        get() =
            // switched because of roborio placement
            Rotation2d.fromDegrees(gyro.pitch.toDouble())

    fun getInclineAngle(): Rotation2d {
        val y = pitch.radians
        val x = roll.radians

        // vector math: treat pitch and roll values as 3d vectors
        // find cross product for normal line of plane,
        // use cosine angle rule for normal line and normal line'
        // s "shadow"
        // (same x and y, but z is 0)
        val pow = (sin(y) * cos(x)).pow(2.0)
        val rad =
            (Math.PI / 2 - acos(sqrt(((cos(y) * sin(x)).pow(2.0) + pow) / (cos(y).pow(2.0) + pow))))
        return Rotation2d.fromRadians(rad).unaryMinus()
    }

    val inclineDirection: Rotation2d
        get() {
            val y = pitch.radians
            val x = roll.radians
            return if (y == 0.0 && x == 0.0) // if robot is perfectly level, set direction to 0
             Rotation2d()
            else {
                // otherwise, angle is arctan of pitch and roll values
                val rad = atan(tan(x) / tan(y))
                if (y < 0) Rotation2d.fromRadians(rad + Math.PI) else Rotation2d.fromRadians(rad)
            }
        }
}
