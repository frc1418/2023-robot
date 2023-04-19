package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ArmConstants

class PivotSubsystem(
    private val pivotMotor: CANSparkMax,
    private val telescopeSensor: TalonFXSensorCollection
) : SubsystemBase() {
    private val pivotEncoder: SparkMaxAbsoluteEncoder =
        pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/arm")
    private val ntPivotPosition = table.getEntry("pivotPosition")
    private val pivotPidController: PIDController
    private var targetPos: Double

    init {
        pivotEncoder.setZeroOffset(ArmConstants.pivotOffset)
        pivotPidController = PIDController(19.0, 0.0, 3.0)
        pivotPidController.setTolerance(3.0, 1.0)
        pivotPidController.enableContinuousInput(0.0, 1.0)
        targetPos = pivotEncoder.position
        defaultCommand = moveToTargetContinuously()
    }

    override fun periodic() {
        ntPivotPosition.setDouble(pivotEncoder.position)
    }

    private val armFeedforward: ArmFeedforward
        get() {
            val kG =
                (ArmConstants.startingPivotG +
                    ArmConstants.pivotGPerTelescopeMeter * telescopeSensor.integratedSensorPosition)
            return ArmFeedforward(0.0, kG, 1.0)
        }

    private fun runMotor() {
        pivotMotor.set(
            armFeedforward.calculate(targetPos, 0.0) +
                pivotPidController.calculate(pivotEncoder.position, targetPos)
        )
    }

    /**
     * Mainly used for command groups, where no default command is run
     *
     * @return A command that continuously runs the motor to the set target.
     */
    private fun moveToTargetContinuously(): Command = run { runMotor() }

    private fun moveToTargetUntilThere(): Command =
        run { runMotor() }.until { pivotPidController.atSetpoint() }

    /**
     * @param pos Position to move arm
     * @return A command that moves arm to pos and keeps it there. This command does not end
     */
    fun moveToTargetContinuously(pos: Double): Command =
        runOnce { setTarget(pos) }.andThen(moveToTargetContinuously())

    /**
     * @param pos Position for the arm to move to
     * @return A command that moves the arm to the setpoint, finishing when the arm is at the
     *   setpoint
     */
    fun moveToTargetUntilThere(pos: Double): Command =
        runOnce { setTarget(pos) }.andThen(moveToTargetUntilThere())

    /**
     * @param pos Position for arm to move to
     * @return A command that sets the position of the arm and ends, allowing the default command to
     *   control movement
     */
    fun moveToTarget(pos: Double): Command = runOnce { setTarget(pos) }

    private fun setTarget(targetPos: Double) {
        pivotPidController.reset()
        this.targetPos = targetPos
    }
}
