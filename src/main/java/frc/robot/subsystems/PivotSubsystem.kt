package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDSubsystem
import frc.robot.Constants.ArmConstants

class PivotSubsystem(
    private val pivotMotor: CANSparkMax,
    private val telescopeSensor: TalonFXSensorCollection
) : PIDSubsystem(PIDController(19.0, 0.0, 0.0), 0.0) {
    private val pivotEncoder: SparkMaxAbsoluteEncoder =
        pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/arm")
    private val ntPivotPosition = table.getEntry("pivotPosition")

    init {
        pivotEncoder.setZeroOffset(ArmConstants.pivotOffset)
        controller.setTolerance(3.0)
        controller.enableContinuousInput(0.0, 1.0)
    }

    override fun getMeasurement() = pivotEncoder.position

    override fun periodic() {
        super.periodic()
        ntPivotPosition.setDouble(pivotEncoder.position)
    }

    private val armFeedforward: ArmFeedforward
        get() {
            val kG =
                (ArmConstants.startingPivotG +
                    ArmConstants.pivotGPerTelescopeMeter * telescopeSensor.integratedSensorPosition)
            return ArmFeedforward(0.0, kG, 1.0)
        }

    /**
     * @param pos Position for arm to move to
     * @return A command that sets the position of the arm and ends, allowing the default command to
     *   control movement
     */
    fun setTarget(pos: Double): CommandBase = runOnce { setpoint = pos }

    override fun useOutput(output: Double, setpoint: Double) {
        pivotMotor.set(armFeedforward.calculate(setpoint, 0.0) + output)
    }
}
