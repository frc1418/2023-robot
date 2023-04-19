package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import frc.robot.Constants.WheelConstants

class WheelSubsystem(angleMotor: CANSparkMax, speedMotor: CANSparkMax, ntName: String) {
    private val anglePIDController: SparkMaxPIDController
    private val speedPIDController: SparkMaxPIDController
    private val encoder = angleMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val speedEncoder = speedMotor.encoder
    private val ntSpeedTarget: NetworkTableEntry
    private val ntVelocity: NetworkTableEntry
    private val angleEncoderNtEntry: NetworkTableEntry
    private var wheelSpeedSetpoint = 0.0

    init {
        val ntInstance = NetworkTableInstance.getDefault()
        val table = ntInstance.getTable("/components/drivetrain/$ntName")
        angleEncoderNtEntry = table.getEntry("AngleEncoder")
        ntSpeedTarget = table.getEntry("speedTarget")
        ntVelocity = table.getEntry("wheelVelocity")
        ntSpeedTarget.setDouble(0.0)
        ntVelocity.setDouble(0.0)
        speedEncoder.setPosition(0.0)
        speedEncoder.setPositionConversionFactor(WheelConstants.ROTATIONS_TO_METERS)
        speedEncoder.setVelocityConversionFactor(speedEncoder.positionConversionFactor / 60.0)
        speedPIDController = speedMotor.pidController
        speedPIDController
        speedPIDController.p = 0.0001
        speedPIDController.i = 0.0
        speedPIDController.d = 0.0
        speedPIDController.ff = 0.25
        anglePIDController = angleMotor.pidController
        anglePIDController.setPositionPIDWrappingMinInput(1.0)
        anglePIDController.setPositionPIDWrappingMaxInput(1.0)
        anglePIDController.setOutputRange(-1.0, 1.0)

        // Swerve modules need inverted encoder
        encoder.setInverted(true)
    }

    fun drive(state: SwerveModuleState) {
        val optimizedState =
            SwerveModuleState.optimize(state, Rotation2d.fromRotations(encoder.position))
        wheelSpeedSetpoint = optimizedState.speedMetersPerSecond
        speedPIDController.setReference(wheelSpeedSetpoint, CANSparkMax.ControlType.kVelocity)
        val angle = optimizedState.angle
        anglePIDController.setReference(angle.rotations, CANSparkMax.ControlType.kPosition)
    }

    val swerveModulePosition: SwerveModulePosition
        get() =
            SwerveModulePosition(speedEncoder.position, Rotation2d.fromRotations(encoder.position))

    fun periodic() {
        angleEncoderNtEntry.setDouble(encoder.position)
        ntSpeedTarget.setDouble(wheelSpeedSetpoint)
        ntVelocity.setDouble(speedEncoder.velocity)
    }
}
