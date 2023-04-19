package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.ArmConstants
import kotlin.math.abs

class TelescopeSubsystem(private val telescopeMotor: TalonFX) : SubsystemBase() {
    private val ntInstance = NetworkTableInstance.getDefault()
    private val table = ntInstance.getTable("/components/arm")
    private val ntTelescopeLength = table.getEntry("telescopeLength")

    init {
        telescopeMotor.selectProfileSlot(0, 0)
        telescopeMotor.config_kP(0, 1.2)
        telescopeMotor.config_kI(0, 0.0)
        telescopeMotor.config_kD(0, 0.5)
        telescopeMotor.config_kF(0, 0.0)
        telescopeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0)
        telescopeMotor.setSensorPhase(true)
        telescopeMotor.sensorCollection.setIntegratedSensorPosition(0.0, 0)
    }

    override fun periodic() {
        updateTelescopeLength()
    }

    private fun updateTelescopeLength() {
        ntTelescopeLength.setDouble(
            ArmConstants.telescopeRotationToMeters *
                telescopeMotor.sensorCollection.integratedSensorPosition
        )
    }

    private fun setTelescopeTarget(target: Double) {
        telescopeMotor[ControlMode.Position] = target
    }

    fun moveToTargetAndHold(target: Double): Command =
        runOnce { setTelescopeTarget(target) }.andThen(Commands.waitUntil { atTarget })

    fun moveToTarget(target: Double): Command = runOnce { setTelescopeTarget(target) }

    private val atTarget: Boolean = abs(telescopeMotor.closedLoopError) < CLOSED_LOOP_THRESHOLD

    companion object {
        const val CLOSED_LOOP_THRESHOLD = 5.0
    }
}
